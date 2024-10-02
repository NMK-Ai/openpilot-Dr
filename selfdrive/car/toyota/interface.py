#!/usr/bin/env python3
from cereal import car
from common.conversions import Conversions as CV
from panda import Panda
from selfdrive.car.toyota.values import Ecu, CAR, DBC, ToyotaFlags, CarControllerParams, TSS2_CAR, RADAR_ACC_CAR, NO_DSU_CAR, \
  MIN_ACC_SPEED, EPS_SCALE, EV_HYBRID_CAR, UNSUPPORTED_DSU_CAR, NO_STOP_TIMER_CAR, ANGLE_CONTROL_CAR
from selfdrive.car import STD_CARGO_KG, scale_tire_stiffness, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from common.params import Params

EventName = car.CarEvent.EventName


class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long):
    ret.carName = "toyota"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.toyota)]
    ret.safetyConfigs[0].safetyParam = EPS_SCALE[candidate]

    # وحدة الفرامل على عنوان مختلف لهذه السيارات
    if DBC[candidate]["pt"] == "toyota_new_mc_pt_generated":
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_TOYOTA_ALT_BRAKE

    if candidate in ANGLE_CONTROL_CAR:
      ret.dashcamOnly = True
      ret.steerControlType = car.CarParams.SteerControlType.angle
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_TOYOTA_LTA
    else:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.steerActuatorDelay = 0.12  # تأخير افتراضي، سيارات بريوس لديها تأخير أكبر
    ret.steerLimitTimer = 0.4
    ret.stoppingControl = False  # تبدأ تويوتا في الفرملة أكثر عندما تعتقد أنك تريد التوقف

    stop_and_go = False

    params = Params()
    if candidate == CAR.PRIUS:
      stop_and_go = True
      ret.wheelbase = 2.70
      ret.steerRatio = 15.74   # المواصفات من الطرف إلى الطرف غير معروفة
      tire_stiffness_factor = 0.6371   # تم ضبطه يدويًا
      ret.mass = 3045. * CV.LB_TO_KG + STD_CARGO_KG
      # إعطاء زاوية التوجيه منطقة ميتة فقط لأجهزة استشعار الزاوية السيئة في سيارات بريوس
      if params.get_bool("dp_toyota_prius_bad_angle_tune"):
        ret.steerActuatorDelay = 0.25
        CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, steering_angle_deadzone_deg=0.2)
      else:
        for fw in car_fw:
          if fw.ecu == "eps" and not fw.fwVersion == b'8965B47060\x00\x00\x00\x00\x00\x00':
            ret.steerActuatorDelay = 0.25
            CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, steering_angle_deadzone_deg=0.2)

    elif candidate == CAR.PRIUS_V:
      stop_and_go = True
      ret.wheelbase = 2.78
      ret.steerRatio = 17.4
      tire_stiffness_factor = 0.5533
      ret.mass = 3340. * CV.LB_TO_KG + STD_CARGO_KG

    elif candidate in (CAR.RAV4, CAR.RAV4H):
      stop_and_go = True if (candidate in CAR.RAV4H) else False
      ret.wheelbase = 2.65
      ret.steerRatio = 16.88   # المواصفات هي 14.5 من الطرف إلى الطرف
      tire_stiffness_factor = 0.5533
      ret.mass = 3650. * CV.LB_TO_KG + STD_CARGO_KG  # متوسط بين العادية والهجينة

    elif candidate == CAR.COROLLA:
      ret.wheelbase = 2.70
      ret.steerRatio = 18.27
      tire_stiffness_factor = 0.444  # لم يتم تحسينه بعد
      ret.mass = 2860. * CV.LB_TO_KG + STD_CARGO_KG  # متوسط بين العادية والهجينة

    elif candidate in (CAR.LEXUS_RX, CAR.LEXUS_RXH, CAR.LEXUS_RX_TSS2, CAR.LEXUS_RXH_TSS2):
      stop_and_go = True
      ret.wheelbase = 2.79
      ret.steerRatio = 16.  # المواصفات هي 14.8 من الطرف إلى الطرف
      ret.wheelSpeedFactor = 1.035
      tire_stiffness_factor = 0.5533
      ret.mass = 4481. * CV.LB_TO_KG + STD_CARGO_KG  # متوسط بين الحد الأدنى والأقصى

    elif candidate in (CAR.CHR, CAR.CHRH, CAR.CHR_TSS2, CAR.CHRH_TSS2):
      stop_and_go = True
      ret.wheelbase = 2.63906
      ret.steerRatio = 13.6
      tire_stiffness_factor = 0.7933
      ret.mass = 3300. * CV.LB_TO_KG + STD_CARGO_KG

    elif candidate in (CAR.CAMRY, CAR.CAMRYH, CAR.CAMRY_TSS2, CAR.CAMRYH_TSS2):
      stop_and_go = True
      ret.wheelbase = 2.82448
      ret.steerRatio = 13.7
      tire_stiffness_factor = 0.7933
      ret.mass = 3400. * CV.LB_TO_KG + STD_CARGO_KG  # متوسط بين العادية والهجينة

    elif candidate in (CAR.HIGHLANDER, CAR.HIGHLANDERH, CAR.HIGHLANDER_TSS2, CAR.HIGHLANDERH_TSS2):
      stop_and_go = True
      ret.wheelbase = 2.8194  # متوسط بين 109.8 و 112.2 بوصة
      ret.steerRatio = 16.0
      tire_stiffness_factor = 0.8
      ret.mass = 4516. * CV.LB_TO_KG + STD_CARGO_KG  # متوسط بين العادية والهجينة

    elif candidate in (CAR.AVALON, CAR.AVALON_2019, CAR.AVALONH_2019, CAR.AVALON_TSS2, CAR.AVALONH_TSS2):
      # ابتداءً من 2019، جميع نسخ أفالون تدعم نظام التوقف والانطلاق
      # https://engage.toyota.com/static/images/toyota_safety_sense/TSS_Applicability_Chart.pdf
      stop_and_go = candidate != CAR.AVALON
      ret.wheelbase = 2.82
      ret.steerRatio = 14.8  # تم العثور عليها في https://pressroom.toyota.com/releases/2016+avalon+product+specs.download
      tire_stiffness_factor = 0.7983
      ret.mass = 3505. * CV.LB_TO_KG + STD_CARGO_KG  # متوسط بين العادية والهجينة

    elif candidate in (CAR.RAV4_TSS2, CAR.RAV4_TSS2_2022, CAR.RAV4H_TSS2, CAR.RAV4H_TSS2_2022,
                       CAR.RAV4_TSS2_2023, CAR.RAV4H_TSS2_2023):
      stop_and_go = True
      ret.wheelbase = 2.68986
      ret.steerRatio = 14.3
      tire_stiffness_factor = 0.7933
      ret.mass = 3585. * CV.LB_TO_KG + STD_CARGO_KG  # متوسط بين البنزين والهجين
      ret.lateralTuning.init('pid')
      ret.lateralTuning.pid.kiBP = [0.0]
      ret.lateralTuning.pid.kpBP = [0.0]
      ret.lateralTuning.pid.kpV = [0.6]
      ret.lateralTuning.pid.kiV = [0.1]
      ret.lateralTuning.pid.kf = 0.00007818594

      # سيارات RAV4 TSS2 من 2019+ تستخدم نظامي توجيه مختلفين وتحتاج إلى ضبط محدد.
      # انظر https://github.com/commaai/openpilot/pull/21429#issuecomment-873652891
      for fw in car_fw:
        if fw.ecu == "eps" and (fw.fwVersion.startswith(b'\x02') or fw.fwVersion in [b'8965B42181\x00\x00\x00\x00\x00\x00']):
          ret.lateralTuning.pid.kpV = [0.15]
          ret.lateralTuning.pid.kiV = [0.05]
          ret.lateralTuning.pid.kf = 0.00004
          break

    elif candidate in (CAR.COROLLA_TSS2, CAR.COROLLAH_TSS2):
      stop_and_go = True
      ret.wheelbase = 2.67  # متوسط بين 2.70 للصالون و 2.64 للهاتشباك
      ret.steerRatio = 13.9
      tire_stiffness_factor = 0.444  # لم يتم تحسينه بعد
      ret.mass = 3060. * CV.LB_TO_KG + STD_CARGO_KG

    elif candidate in (CAR.LEXUS_ES_TSS2, CAR.LEXUS_ESH_TSS2, CAR.LEXUS_ESH):
      stop_and_go = True
      ret.wheelbase = 2.8702
      ret.steerRatio = 16.0  # لم يتم تحسينه بعد
      tire_stiffness_factor = 0.444  # لم يتم تحسينه بعد
      ret.mass = 3677. * CV.LB_TO_KG + STD_CARGO_KG  # متوسط بين الحد الأدنى والأقصى

    elif candidate == CAR.SIENNA:
      stop_and_go = True
      ret.wheelbase = 3.03
      ret.steerRatio = 15.5
      tire_stiffness_factor = 0.444
      ret.mass = 4590. * CV.LB_TO_KG + STD_CARGO_KG

    elif candidate in (CAR.LEXUS_IS, CAR.LEXUS_RC):
      ret.wheelbase = 2.79908
      ret.steerRatio = 13.3
      tire_stiffness_factor = 0.444
      ret.mass = 3736.8 * CV.LB_TO_KG + STD_CARGO_KG

    elif candidate == CAR.LEXUS_CTH:
      stop_and_go = True
      ret.wheelbase = 2.60
      ret.steerRatio = 18.6
      tire_stiffness_factor = 0.517
      ret.mass = 3108 * CV.LB_TO_KG + STD_CARGO_KG  # متوسط بين الحد الأدنى والأقصى

    elif candidate in (CAR.LEXUS_NX, CAR.LEXUS_NXH, CAR.LEXUS_NX_TSS2, CAR.LEXUS_NXH_TSS2):
      stop_and_go = True
      ret.wheelbase = 2.66
      ret.steerRatio = 14.7
      tire_stiffness_factor = 0.444  # لم يتم تحسينه بعد
      ret.mass = 4070 * CV.LB_TO_KG + STD_CARGO_KG

    elif candidate == CAR.PRIUS_TSS2:
      stop_and_go = True
      ret.wheelbase = 2.70002  # من المواصفات على الإنترنت لتويوتا.
      ret.steerRatio = 13.4   # نسبة التوجيه الفعلية من بريوس القديمة
      tire_stiffness_factor = 0.6371   # تم ضبطه يدويًا
      ret.mass = 3115. * CV.LB_TO_KG + STD_CARGO_KG

    elif candidate == CAR.MIRAI:
      stop_and_go = True
      ret.wheelbase = 2.91
      ret.steerRatio = 14.8
      tire_stiffness_factor = 0.8
      ret.mass = 4300. * CV.LB_TO_KG + STD_CARGO_KG

    elif candidate in (CAR.ALPHARD_TSS2, CAR.ALPHARDH_TSS2):
      stop_and_go = True
      ret.wheelbase = 3.00
      ret.steerRatio = 14.2
      tire_stiffness_factor = 0.444
      ret.mass = 4305. * CV.LB_TO_KG + STD_CARGO_KG

    CarInterfaceBase.dp_lat_tune_collection(candidate, ret.latTuneCollection, steering_angle_deadzone_deg = 0.0)
    CarInterfaceBase.configure_dp_tune(ret.lateralTuning, ret.latTuneCollection)

    ret.centerToFront = ret.wheelbase * 0.44

    # TODO: ابدأ من الصلابة العرضية للإطارات المستمدة تجريبيًا لسيارة سيفيك واضبطها وفقًا
    # للوزن وموضع مركز الجاذبية، بحيث يكون لجميع السيارات سلوكيات ديناميكية متشابهة تقريبًا
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.enableBsm = 0x3F6 in fingerprint[0] and candidate in TSS2_CAR
    # اكتشاف smartDSU، الذي يعترض ACC_CMD من DSU مما يسمح لـ openpilot بإرساله
    smartDsu = 0x2FF in fingerprint[0]
    # في سيارات TSS2، تقوم الكاميرا بالتحكم الطولي
    found_ecus = [fw.ecu for fw in car_fw]
    ret.enableDsu = len(found_ecus) > 0 and Ecu.dsu not in found_ecus and candidate not in (NO_DSU_CAR | UNSUPPORTED_DSU_CAR) and not smartDsu
    ret.enableGasInterceptor = 0x201 in fingerprint[0]
    # إذا تم اكتشاف smartDSU، يمكن لـ openpilot إرسال ACC_CMD (وسيقوم smartDSU بحظره من DSU) أو لا (سيظل DSU "متصلًا")
    ret.openpilotLongitudinalControl = smartDsu or ret.enableDsu or candidate in (TSS2_CAR - RADAR_ACC_CAR)
    ret.autoResumeSng = ret.openpilotLongitudinalControl and candidate in NO_STOP_TIMER_CAR

    if int(Params().get("dp_atl").decode('utf-8')) == 1:
      ret.openpilotLongitudinalControl = False

    if candidate == CAR.CHR_TSS2:
      ret.enableBsm = True

    if not ret.openpilotLongitudinalControl:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_TOYOTA_STOCK_LONGITUDINAL

    # لا يمكننا استخدام البصمة لاكتشاف هذا بشكل موثوق، لأن إشارة دواسة الوقود للسيارات الكهربائية يمكن أن تستغرق بضع ثوانٍ للظهور
    if candidate in EV_HYBRID_CAR:
      ret.flags |= ToyotaFlags.HYBRID.value

    # الحد الأدنى للسرعة لتفعيل ACC. إذا كانت السيارة تدعم التوقف والانطلاق، فقم بتعيين سرعة التفعيل
    # إلى قيمة سالبة، لذلك لن تهم.
    ret.minEnableSpeed = -1. if (stop_and_go or ret.enableGasInterceptor) else MIN_ACC_SPEED

    tune = ret.longitudinalTuning
    tune.deadzoneBP = [0., 9.]
    tune.deadzoneV = [.0, .15]
    إذا كانت السيارة مرشحة ضمن TSS2_CAR أو ret.enableGasInterceptor:
      tune.kpBP = [0., 5., 20.]
      tune.kpV = [1.3, 1.0, 0.7]
      tune.kiBP = [0., 3., 4., 5., 12., 20., 23., 40.]
      tune.kiV = [.08, .16, .26, .215, .20, .166, .1, .006]
      إذا كانت السيارة مرشحة ضمن TSS2_CAR:
        # ret.vEgoStopping = 0.3  # السيارة تكون بين 0.1 إلى 0.2 عندما تبدأ السيارة في طلب التسارع المتوقف
        ret.vEgoStarting = 0.1  # يجب أن تكون أكبر أو تساوي vEgoStopping
        # ret.stopAccel = -0.1  # تطلب تويوتا -0.4 عند التوقف
        ret.stoppingDecelRate = 0.04  # الوصول إلى هدف التوقف بسلاسة - يبدو أن الأمر يستغرق 0.5 ثانية للانتقال من 0 إلى -0.4
        # ret.longitudinalActuatorDelayLowerBound = 0.3
        # ret.longitudinalActuatorDelayUpperBound = 0.3
        ### الافتراضي ###
        # ret.vEgoStopping = 0.25
        # ret.vEgoStarting = 0.25
        # ret.stoppingDecelRate = 0.3  # الوصول إلى هدف التوقف بسلاسة
    else:
      tune.kpBP = [0., 5., 35.]
      tune.kiBP = [0., 35.]
      tune.kpV = [3.6, 2.4, 1.5]
      tune.kiV = [0.54, 0.36]

    return ret

  # يعيد car.CarState
  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)

    # الأحداث
    events = self.create_common_events(ret)

    إذا كان التحكم الطولي في openpilot مفعلًا في هذا السيارة:
      إذا كان ret.cruiseState.standstill ولم يتم الضغط على الفرامل ولم يتم تفعيل جهاز اعتراض الوقود:
        events.add(EventName.resumeRequired)
      إذا كانت self.CS.low_speed_lockout:
        events.add(EventName.lowSpeedLockout)
      إذا كانت سرعة ret.vEgo أقل من الحد الأدنى للسرعة للتفعيل:
        events.add(EventName.belowEngageSpeed)
        إذا كان تسارع c.actuators.accel أكبر من 0.3:
          # بعض الهامش على المحرك لتجنب التفعيل الزائف للإلغاء أثناء التوقف
          events.add(EventName.speedTooLow)
        إذا كانت سرعة ret.vEgo أقل من 0.001:
          # أثناء التوقف التام، يتم إرسال تنبيه للمستخدم
          events.add(EventName.manualRestart)

    ret.events = events.to_msg()

    return ret

  # تمرير car.CarControl
  # يتم استدعاؤه @ 100Hz
  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos, self.dragonconf)
