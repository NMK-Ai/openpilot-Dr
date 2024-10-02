from cereal import car
from selfdrive.car.chrysler.values import RAM_CARS

GearShifter = car.CarState.GearShifter
VisualAlert = car.CarControl.HUDControl.VisualAlert

def create_lkas_hud(packer, CP, lkas_active, hud_alert, hud_count, car_model, auto_high_beam):
  # LKAS_HUD - يتحكم في الأيقونة التي تظهر للحفاظ على المسار

  # == اللون ==
  # 0 مخفي؟
  # 1 أبيض
  # 2 أخضر
  # 3 مغادرة المسار (ldw)

  # == الخطوط ==
  # 03 خطوط بيضاء
  # 04 خطوط رمادية
  # 09 المسار الأيسر قريب
  # 0A المسار الأيمن قريب
  # 0B المسار الأيسر قريب جدًا
  # 0C المسار الأيمن قريب جدًا
  # 0D تقاطع المسار الأيسر
  # 0E تقاطع المسار الأيمن

  # == التنبيهات ==
  # 7 عادي
  # 6 مغادرة المسار، ضع يديك على عجلة القيادة

  color = 2 if lkas_active else 1
  lines = 3 if lkas_active else 0
  alerts = 7 if lkas_active else 0

  if hud_count < (1 * 4):  # first 3 seconds, 4Hz
    alerts = 1

  if hud_alert in (VisualAlert.ldw, VisualAlert.steerRequired):
    color = 4
    lines = 0
    alerts = 6

  values = {
    "LKAS_ICON_COLOR": color,
    "CAR_MODEL": car_model,
    "LKAS_LANE_LINES": lines,
    "LKAS_ALERTS": alerts,
  }

  if CP.carFingerprint in RAM_CARS:
    values['AUTO_HIGH_BEAM_ON'] = auto_high_beam

  return packer.make_can_msg("DAS_6", 0, values)


def create_lkas_command(packer, CP, apply_steer, lkas_control_bit):
  # LKAS_COMMAND إشارة الحفاظ على المسار لتدوير عجلة القيادة
  enabled_val = 2 if CP.carFingerprint in RAM_CARS else 1
  values = {
    "STEERING_TORQUE": apply_steer,
    "LKAS_CONTROL_BIT": enabled_val if lkas_control_bit else 0,
  }
  return packer.make_can_msg("LKAS_COMMAND", 0, values)


def create_cruise_buttons(packer, frame, bus, cancel=False, resume=False):
  values = {
    "ACC_Cancel": cancel,
    "ACC_Resume": resume,
    "COUNTER": frame % 0x10,
  }
  return packer.make_can_msg("CRUISE_BUTTONS", bus, values)