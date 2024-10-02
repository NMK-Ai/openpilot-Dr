#!/usr/bin/env python3
from cereal import car
from panda import Panda
from selfdrive.car import STD_CARGO_KG, get_safety_config
from selfdrive.car.chrysler.values import CAR, RAM_HD, RAM_DT, RAM_CARS, ChryslerFlags
from selfdrive.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long):
    ret.carName = "chrysler"
    ret.dashcamOnly = candidate in RAM_HD

    # تحليل الرادار يحتاج إلى بعض العمل، راجع https://github.com/commaai/openpilot/issues/26842
    ret.radarUnavailable = True # DBC[candidate]['radar'] غير موجود
    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.4

    # safety config
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.chrysler)]
    if candidate in RAM_HD:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_CHRYSLER_RAM_HD
    elif candidate in RAM_DT:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_CHRYSLER_RAM_DT

    ret.minSteerSpeed = 3.8  # م/ث
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    if candidate not in RAM_CARS:
      # الإصدارات الأحدث من البرمجيات الثابتة (FW) القياسية على المنصات التالية، أو التي يتم تحديثها بواسطة الوكيل على المنصات الأقدم، لديها سرعة توجيه دنيا أعلى.
      new_eps_platform = candidate in (CAR.PACIFICA_2019_HYBRID, CAR.PACIFICA_2020, CAR.JEEP_CHEROKEE_2019)
      new_eps_firmware = any(fw.ecu == 'eps' and fw.fwVersion[:4] >= b"6841" for fw in car_fw)
      if new_eps_platform or new_eps_firmware:
        ret.flags |= ChryslerFlags.HIGHER_MIN_STEERING_SPEED.value

    # كرايسلر
    if candidate in (CAR.PACIFICA_2017_HYBRID, CAR.PACIFICA_2018, CAR.PACIFICA_2018_HYBRID, CAR.PACIFICA_2019_HYBRID, CAR.PACIFICA_2020):
      ret.mass = 2242. + STD_CARGO_KG
      ret.wheelbase = 3.089
      ret.steerRatio = 16.2  # باسيفيكا هايبرد 2017

      ret.lateralTuning.init('pid')
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[9., 20.], [9., 20.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.15, 0.30], [0.03, 0.05]]
      ret.lateralTuning.pid.kf = 0.00006

    # جيب
    elif candidate in (CAR.JEEP_CHEROKEE, CAR.JEEP_CHEROKEE_2019):
      ret.mass = 1778 + STD_CARGO_KG
      ret.wheelbase = 2.71
      ret.steerRatio = 16.7
      ret.steerActuatorDelay = 0.2

      ret.lateralTuning.init('pid')
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[9., 20.], [9., 20.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.15, 0.30], [0.03, 0.05]]
      ret.lateralTuning.pid.kf = 0.00006

    # رام
    elif candidate == CAR.RAM_1500:
      ret.steerActuatorDelay = 0.2
      ret.wheelbase = 3.88
      ret.steerRatio = 16.3
      ret.mass = 2493. + STD_CARGO_KG
      ret.minSteerSpeed = 14.5
      # الإصدارات الأقدم من برمجيات EPS تسمح بالتوجيه إلى الصفر
      if any(fw.ecu == 'eps' and fw.fwVersion[:4] <= b"6831" for fw in car_fw):
        ret.minSteerSpeed = 0.

    elif candidate == CAR.RAM_HD:
      ret.steerActuatorDelay = 0.2
      ret.wheelbase = 3.785
      ret.steerRatio = 15.61
      ret.mass = 3405. + STD_CARGO_KG
      ret.minSteerSpeed = 16
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, 1.0, False)

    else:
      raise ValueError(f"Unsupported car: {candidate}")

    CarInterfaceBase.dp_lat_tune_collection(candidate, ret.latTuneCollection)
    CarInterfaceBase.configure_dp_tune(ret.lateralTuning, ret.latTuneCollection)

    if ret.flags & ChryslerFlags.HIGHER_MIN_STEERING_SPEED:
      # TODO: السماح لهذه السيارات بالتوجيه حتى 13 م/ث إذا كانت مفعلة بالفعل.
      ret.minSteerSpeed = 17.5  # م/ث، 17 عند الصعود، و13 عند النزول بعد التفعيل.

    ret.centerToFront = ret.wheelbase * 0.44
    ret.enableBsm = 720 in fingerprint[0]

    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)

    # الأحداث
    events = self.create_common_events(ret, extra_gears=[car.CarState.GearShifter.low])

    # منطق الهيسترسيس لتنبيه التوجيه عند السرعات المنخفضة
    if self.CP.minSteerSpeed > 0. and ret.vEgo < (self.CP.minSteerSpeed + 0.5):
      self.low_speed_alert = True
    elif ret.vEgo > (self.CP.minSteerSpeed + 1.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    ret.events = events.to_msg()

    return ret

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)