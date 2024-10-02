from cereal import car
from common.numpy_fast import clip, interp
from selfdrive.car import apply_meas_steer_torque_limits, create_gas_interceptor_command, make_can_msg
from selfdrive.car.toyota.toyotacan import create_steer_command, create_ui_command, \
                                           create_accel_command, create_acc_cancel_command, \
                                           create_fcw_command, create_lta_steer_command
from selfdrive.car.toyota.values import CAR, STATIC_DSU_MSGS, NO_STOP_TIMER_CAR, TSS2_CAR, \
                                        MIN_ACC_SPEED, PEDAL_TRANSITION, CarControllerParams, \
                                        UNSUPPORTED_DSU_CAR
from opendbc.can.packer import CANPacker
from common.conversions import Conversions as CV
from common.params import Params

VisualAlert = car.CarControl.HUDControl.VisualAlert

# تحدث أخطاء EPS إذا قمت بتطبيق عزم دوران بينما تكون سرعة التوجيه أعلى من 100 درجة/ثانية لفترة طويلة
MAX_STEER_RATE = 100  # الحد الأقصى لمعدل التوجيه بالدرجات/ثانية
MAX_STEER_RATE_FRAMES = 18  # عدد إطارات التحكم المطلوبة قبل أن يتم قطع عزم الدوران

# يسمح نظام EPS بعزم دوران المستخدم فوق العتبة لمدة 50 إطارًا قبل حدوث خطأ دائم
MAX_USER_TORQUE = 500

GearShifter = car.CarState.GearShifter
UNLOCK_CMD = b'\x40\x05\x30\x11\x00\x40\x00\x00'
LOCK_CMD = b'\x40\x05\x30\x11\x00\x80\x00\x00'
LOCK_AT_SPEED = 10 * CV.KPH_TO_MS

class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.torque_rate_limits = CarControllerParams(self.CP)
    self.frame = 0
    self.last_steer = 0
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.steer_rate_counter = 0

    self.steer_rate_counter = 0

    self.packer = CANPacker(dbc_name)
    self.gas = 0
    self.accel = 0

    # dp
    self.dp_toyota_sng = False

    self.dp_toyota_auto_lock = False
    self.dp_toyota_auto_unlock = False
    self.last_gear = GearShifter.park
    self.lock_once = False
    self.lat_controller_type = None
    self.lat_controller_type_prev = None

  def update(self, CC, CS, now_nanos, dragonconf):
    if dragonconf is not None:
      self.dp_toyota_sng = dragonconf.dpToyotaSng
      self.dp_toyota_auto_lock = dragonconf.dpToyotaAutoLock
      self.dp_toyota_auto_unlock = dragonconf.dpToyotaAutoUnlock
    self.lat_controller_type = CC.latController
    if self.lat_controller_type != self.lat_controller_type_prev:
      self.torque_rate_limits.update(CC.latController)
    self.lat_controller_type_prev = self.lat_controller_type

    self.dp_toyota_change5speed = Params().get_bool("dp_toyota_change5speed")
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    # دواسة الوقود والفرامل
    if self.CP.enableGasInterceptor and CC.longActive:
      MAX_INTERCEPTOR_GAS = 0.5
      # دواسة الوقود في سيارة RAV4 حساسة للغاية
      if self.CP.carFingerprint in (CAR.RAV4, CAR.RAV4H, CAR.HIGHLANDER, CAR.HIGHLANDERH):
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.15, 0.3, 0.0])
      elif self.CP.carFingerprint in (CAR.COROLLA,):
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.3, 0.4, 0.0])
      else:
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.4, 0.5, 0.0])
      # تعويض للحركة البطيئة وفرملة الرياح
      pedal_offset = interp(CS.out.vEgo, [0.0, 2.3, MIN_ACC_SPEED + PEDAL_TRANSITION], [-.4, 0.0, 0.2])
      pedal_command = PEDAL_SCALE * (actuators.accel + pedal_offset)
      interceptor_gas_cmd = clip(pedal_command, 0., MAX_INTERCEPTOR_GAS)
    else:
      interceptor_gas_cmd = 0.
    pcm_accel_cmd = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

    # عزم دوران التوجيه
    new_steer = int(round(actuators.steer * CarControllerParams.STEER_MAX))
    apply_steer = apply_meas_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, self.torque_rate_limits)

    # العد حتى MAX_STEER_RATE_FRAMES، وفي هذه المرحلة نحتاج إلى قطع عزم الدوران لتجنب حدوث خطأ في التوجيه
    if lat_active and abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE:
      self.steer_rate_counter += 1
    else:
      self.steer_rate_counter = 0

    apply_steer_req = 1
    if not lat_active:
      apply_steer = 0
      apply_steer_req = 0
    elif self.steer_rate_counter > MAX_STEER_RATE_FRAMES:
      apply_steer_req = 0
      self.steer_rate_counter = 0

    # لا تقم بتفعيل التوجيه مع نظام LKA في السيارات التي تدعم فقط نظام LTA
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      apply_steer = 0
      apply_steer_req = 0

    # TODO: من المحتمل أنه يمكن حذف هذا. يستخدم CS.pcm_acc_status إشارة مختلفة
    # عن CS.cruiseState.enabled. تأكد من أنها ليست مختلفة بشكل كبير
    if not CC.enabled and CS.pcm_acc_status:
      pcm_cancel_cmd = 1

    # عند الدخول في حالة التوقف التام، أرسل طلب التوقف التام
    if CS.out.standstill and not self.last_standstill and (self.CP.carFingerprint not in NO_STOP_TIMER_CAR or self.CP.enableGasInterceptor):
      self.standstill_req = True
    if CS.pcm_acc_status != 8:
      # دخل نظام PCM في حالة التوقف التام أو تم تعطيله
      self.standstill_req = False
    self.standstill_req = False if self.dp_toyota_sng else self.standstill_req

    self.last_steer = apply_steer
    self.last_standstill = CS.out.standstill

    can_sends = []

    # dp - منطق القفل / الفتح التلقائي للأبواب
    # شكرًا لـ AlexandreSato و cydia2020
    # https://github.com/AlexandreSato/animalpilot/blob/personal/doors.py
    if self.dp_toyota_auto_lock or self.dp_toyota_auto_unlock:
      gear = CS.out.gearShifter
      if self.last_gear != gear and gear == GearShifter.park:
        if self.dp_toyota_auto_unlock:
          can_sends.append(make_can_msg(0x750, UNLOCK_CMD, 0))
        if self.dp_toyota_auto_lock:
          self.lock_once = False
      elif self.dp_toyota_auto_lock and not CS.out.doorOpen and gear == GearShifter.drive and not self.lock_once and CS.out.vEgo >= LOCK_AT_SPEED:
        can_sends.append(make_can_msg(0x750, LOCK_CMD, 0))
        self.lock_once = True
      self.last_gear = gear

    # *** رسائل التحكم ***
    # print("steer {0} {1} {2} {3}".format(apply_steer, min_lim, max_lim, CS.steer_torque_motor)

    # تتبع CAN في سيارات تويوتا يظهر هذه الرسالة بتردد 42 هرتز، مع إضافة العداد بالتناوب 1 و 2؛
    # إرسالها بتردد 100 هرتز يبدو أنه يسمح بحد أعلى للتردد، حيث يبدو أن الحد الأقصى يتم فرضه
    # على الرسائل المتتالية
    can_sends.append(create_steer_command(self.packer, apply_steer, apply_steer_req))
    if self.frame % 2 == 0 and self.CP.carFingerprint in TSS2_CAR:
      can_sends.append(create_lta_steer_command(self.packer, 0, 0, self.frame // 2))

    # وضع LTA. قم بتعيين ret.steerControlType = car.CarParams.SteerControlType.angle وإضافة 0x191 إلى القائمة البيضاء في جهاز الباندا
    # إذا كانت قيمة self.frame % 2 == 0:
    #   can_sends.append(create_steer_command(self.packer, 0, 0, self.frame // 2))
    #   can_sends.append(create_lta_steer_command(self.packer, actuators.steeringAngleDeg, apply_steer_req, self.frame // 2))
    if (self.frame % 3 == 0 and self.CP.openpilotLongitudinalControl) or pcm_cancel_cmd:
      lead = hud_control.leadVisible or CS.out.vEgo < 12.  # at low speed we always assume the lead is present so ACC can be engaged

      # تستخدم سيارة لكزس IS رسالة إلغاء مختلفة
      if pcm_cancel_cmd and self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
        can_sends.append(create_acc_cancel_command(self.packer))
      elif self.CP.openpilotLongitudinalControl:
        can_sends.append(create_accel_command(self.packer, pcm_accel_cmd, pcm_cancel_cmd, self.standstill_req, lead, CS.acc_type, CS.distance, self.dp_toyota_change5speed))
        self.accel = pcm_accel_cmd
      else:
        can_sends.append(create_accel_command(self.packer, 0, pcm_cancel_cmd, False, lead, CS.acc_type, CS.distance, self.dp_toyota_change5speed))

    if self.frame % 2 == 0 and self.CP.enableGasInterceptor and self.CP.openpilotLongitudinalControl:
      # أرسل قيمة صفرية تمامًا إذا كانت قيمة أمر دواسة الوقود صفرًا. سيقوم جهاز Interceptor بإرسال القيمة القصوى بين القيمة المقروءة وأمر دواسة الوقود.
      # هذا يمنع إعادة ضبط غير متوقعة لنطاق الدواسة
      can_sends.append(create_gas_interceptor_command(self.packer, interceptor_gas_cmd, self.frame // 2))
      self.gas = interceptor_gas_cmd

    if self.CP.carFingerprint != CAR.PRIUS_V:
      # رسالة واجهة المستخدم تكون بتردد 1 هرتز، لكن نرسلها فورًا إذا:
      # - هناك شيء لعرضه
      # - هناك شيء للتوقف عن عرضه
      fcw_alert = hud_control.visualAlert == VisualAlert.fcw
      steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)

      send_ui = False
      if ((fcw_alert or steer_alert) and not self.alert_active) or \
         (not (fcw_alert or steer_alert) and self.alert_active):
        send_ui = True
        self.alert_active = not self.alert_active
      elif pcm_cancel_cmd:
        # إجبار PCM على فك الارتباط يتسبب في إصدار صوت خطأ سيئ، لذا قم بتشغيل صوت جيد بدلاً من ذلك
        send_ui = True

      if self.frame % 20 == 0 or send_ui:
        can_sends.append(create_ui_command(self.packer, steer_alert, pcm_cancel_cmd, hud_control.leftLaneVisible,
                                           hud_control.rightLaneVisible, hud_control.leftLaneDepart,
                                           hud_control.rightLaneDepart, CC.latActive, CS.lkas_hud))

      if (self.frame % 100 == 0 or send_ui) and self.CP.enableDsu:
        can_sends.append(create_fcw_command(self.packer, fcw_alert))

    # *** رسائل ثابتة ***
    for addr, cars, bus, fr_step, vl in STATIC_DSU_MSGS:
      if self.frame % fr_step == 0 and self.CP.enableDsu and self.CP.carFingerprint in cars:
        can_sends.append(make_can_msg(addr, vl, bus))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / CarControllerParams.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas

    self.frame += 1
    return new_actuators, can_sends
