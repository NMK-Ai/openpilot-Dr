# This Python file uses the following encoding: utf-8
# -*- coding: utf-8 -*-
from common.i18n import events
_ = events()

import math
import os
from enum import IntEnum
from typing import Dict, Union, Callable, List, Optional

from cereal import log, car
import cereal.messaging as messaging
from common.conversions import Conversions as CV
from common.realtime import DT_CTRL
from selfdrive.locationd.calibrationd import MIN_SPEED_FILTER
from system.version import get_short_branch

AlertSize = log.ControlsState.AlertSize
AlertStatus = log.ControlsState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
EventName = car.CarEvent.EventName


# Alert priorities
class Priority(IntEnum):
  LOWEST = 0
  LOWER = 1
  LOW = 2
  MID = 3
  HIGH = 4
  HIGHEST = 5


# Event types
class ET:
  ENABLE = 'enable'
  PRE_ENABLE = 'preEnable'
  OVERRIDE_LATERAL = 'overrideLateral'
  OVERRIDE_LONGITUDINAL = 'overrideLongitudinal'
  NO_ENTRY = 'noEntry'
  WARNING = 'warning'
  USER_DISABLE = 'userDisable'
  SOFT_DISABLE = 'softDisable'
  IMMEDIATE_DISABLE = 'immediateDisable'
  PERMANENT = 'permanent'


# get event name from enum
EVENT_NAME = {v: k for k, v in EventName.schema.enumerants.items()}


class Events:
  def __init__(self):
    self.events: List[int] = []
    self.static_events: List[int] = []
    self.events_prev = dict.fromkeys(EVENTS.keys(), 0)

  @property
  def names(self) -> List[int]:
    return self.events

  def __len__(self) -> int:
    return len(self.events)

  def add(self, event_name: int, static: bool=False) -> None:
    if static:
      self.static_events.append(event_name)
    self.events.append(event_name)

  def clear(self) -> None:
    self.events_prev = {k: (v + 1 if k in self.events else 0) for k, v in self.events_prev.items()}
    self.events = self.static_events.copy()

  def any(self, event_type: str) -> bool:
    return any(event_type in EVENTS.get(e, {}) for e in self.events)

  def create_alerts(self, event_types: List[str], callback_args=None):
    if callback_args is None:
      callback_args = []

    ret = []
    for e in self.events:
      types = EVENTS[e].keys()
      for et in event_types:
        if et in types:
          alert = EVENTS[e][et]
          if not isinstance(alert, Alert):
            alert = alert(*callback_args)

          if DT_CTRL * (self.events_prev[e] + 1) >= alert.creation_delay:
            alert.alert_type = f"{EVENT_NAME[e]}/{et}"
            alert.event_type = et
            ret.append(alert)
    return ret

  def add_from_msg(self, events):
    for e in events:
      self.events.append(e.name.raw)

  def to_msg(self):
    ret = []
    for event_name in self.events:
      event = car.CarEvent.new_message()
      event.name = event_name
      for event_type in EVENTS.get(event_name, {}):
        setattr(event, event_type, True)
      ret.append(event)
    return ret


class Alert:
  def __init__(self,
               alert_text_1: str,
               alert_text_2: str,
               alert_status: log.ControlsState.AlertStatus,
               alert_size: log.ControlsState.AlertSize,
               priority: Priority,
               visual_alert: car.CarControl.HUDControl.VisualAlert,
               audible_alert: car.CarControl.HUDControl.AudibleAlert,
               duration: float,
               alert_rate: float = 0.,
               creation_delay: float = 0.):

    self.alert_text_1 = alert_text_1
    self.alert_text_2 = alert_text_2
    self.alert_status = alert_status
    self.alert_size = alert_size
    self.priority = priority
    self.visual_alert = visual_alert
    self.audible_alert = audible_alert

    self.duration = int(duration / DT_CTRL)

    self.alert_rate = alert_rate
    self.creation_delay = creation_delay

    self.alert_type = ""
    self.event_type: Optional[str] = None

  def __str__(self) -> str:
    return f"{self.alert_text_1}/{self.alert_text_2} {self.priority} {self.visual_alert} {self.audible_alert}"

  def __gt__(self, alert2) -> bool:
    if not isinstance(alert2, Alert):
      return False
    return self.priority > alert2.priority


class NoEntryAlert(Alert):
  def __init__(self, alert_text_2: str,
               alert_text_1: str = _("القائد الآلي غير متوفر"),
               visual_alert: car.CarControl.HUDControl.VisualAlert=VisualAlert.none):
    super().__init__(alert_text_1, alert_text_2, AlertStatus.normal,
                     AlertSize.mid, Priority.LOW, visual_alert,
                     AudibleAlert.refuse, 3.)


class SoftDisableAlert(Alert):
  def __init__(self, alert_text_2: str):
    super().__init__(_("قم بالتحكم بالسيارة فوراً"), alert_text_2,
                     AlertStatus.userPrompt, AlertSize.full,
                     Priority.MID, VisualAlert.steerRequired,
                     AudibleAlert.warningSoft, 2.),


# less harsh version of SoftDisable, where the condition is user-triggered
class UserSoftDisableAlert(SoftDisableAlert):
  def __init__(self, alert_text_2: str):
    super().__init__(alert_text_2),
    self.alert_text_1 = _("سيتم فصل القائد الآلي")


class ImmediateDisableAlert(Alert):
  def __init__(self, alert_text_2: str):
    super().__init__(_("قم بالتحكم بالسيارة فوراً"), alert_text_2,
                     AlertStatus.critical, AlertSize.full,
                     Priority.HIGHEST, VisualAlert.steerRequired,
                     AudibleAlert.warningImmediate, 4.),


class EngagementAlert(Alert):
  def __init__(self, audible_alert: car.CarControl.HUDControl.AudibleAlert):
    super().__init__("", "",
                     AlertStatus.normal, AlertSize.none,
                     Priority.MID, VisualAlert.none,
                     audible_alert, .2),


class NormalPermanentAlert(Alert):
  def __init__(self, alert_text_1: str, alert_text_2: str = "", duration: float = 0.2, priority: Priority = Priority.LOWER, creation_delay: float = 0.):
    super().__init__(alert_text_1, alert_text_2,
                     AlertStatus.normal, AlertSize.mid if len(alert_text_2) else AlertSize.small,
                     priority, VisualAlert.none, AudibleAlert.none, duration, creation_delay=creation_delay),


class StartupAlert(Alert):
  def __init__(self, alert_text_1: str, alert_text_2: str = _("أبق يديك على الدركسون وعينيك على الطريق"), alert_status=AlertStatus.normal):
    super().__init__(alert_text_1, alert_text_2,
                     alert_status, AlertSize.mid,
                     Priority.LOWER, VisualAlert.none, AudibleAlert.none, 10.),


# ********** helper functions **********
def get_display_speed(speed_ms: float, metric: bool) -> str:
  speed = int(round(speed_ms * (CV.MS_TO_KPH if metric else CV.MS_TO_MPH)))
  unit = 'km/h' if metric else 'mph'
  return f"{speed} {unit}"


# ********** alert callback functions **********

AlertCallbackType = Callable[[car.CarParams, car.CarState, messaging.SubMaster, bool, int], Alert]


def soft_disable_alert(alert_text_2: str) -> AlertCallbackType:
  def func(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
    if soft_disable_time < int(0.5 / DT_CTRL):
      return ImmediateDisableAlert(alert_text_2)
    return SoftDisableAlert(alert_text_2)
  return func

def user_soft_disable_alert(alert_text_2: str) -> AlertCallbackType:
  def func(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
    if soft_disable_time < int(0.5 / DT_CTRL):
      return ImmediateDisableAlert(alert_text_2)
    return UserSoftDisableAlert(alert_text_2)
  return func

def startup_master_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  branch = get_short_branch("")  # Ensure get_short_branch is cached to avoid lags on startup
  if "REPLAY" in os.environ:
    branch = "replay"

  return StartupAlert(_("تحذير: هذا الفرع غير مجرب"), branch, alert_status=AlertStatus.userPrompt)

def below_engage_speed_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  return NoEntryAlert(f"قد فوق {get_display_speed(CP.minEnableSpeed, metric)} للتفعيل")


def below_steer_speed_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  return Alert(
    _("المقود غير متوفر أدنى من %s") % get_display_speed(CP.minSteerSpeed, metric),
    "",
    AlertStatus.userPrompt, AlertSize.small,
    Priority.MID, VisualAlert.steerRequired, AudibleAlert.prompt, 0.4)


def calibration_incomplete_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  return Alert(
    _("قيد المعايرة: %d%%") % sm['liveCalibration'].calPerc,
    _("قد فوق %s") % get_display_speed(MIN_SPEED_FILTER, metric),
    AlertStatus.normal, AlertSize.mid,
    Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .2)


def no_gps_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  return Alert(
    _("إشارة GPS ضعيفة"),
    _("مشكلة في الإشارة إذا كانت السماء مرئية"),
    AlertStatus.normal, AlertSize.mid,
    Priority.LOWER, VisualAlert.none, AudibleAlert.none, .2, creation_delay=300.)

# *** debug alerts ***

def out_of_space_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  full_perc = round(100. - sm['deviceState'].freeSpacePercent)
  return NormalPermanentAlert(_("الذاكرة غير متاحة"), _("%s%% ممتلئة") % full_perc)


def posenet_invalid_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  mdl = sm['modelV2'].velocity.x[0] if len(sm['modelV2'].velocity.x) else math.nan
  err = CS.vEgo - mdl
  msg = f"خطاء في السرعة: {err:.1f} ج/ث"
  return NoEntryAlert(msg, alert_text_1=_("Posenet Speed Invalid"))


def process_not_running_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  not_running = [p.name for p in sm['managerState'].processes if not p.running and p.shouldBeRunning]
  msg = ', '.join(not_running)
  return NoEntryAlert(msg, alert_text_1=_("العملية لا تعمل"))


def comm_issue_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  bs = [s for s in sm.data.keys() if not sm.all_checks([s, ])]
  msg = ', '.join(bs[:4])  # can't fit too many on one line
  return NoEntryAlert(msg, alert_text_1=_("مشكلة في الاتصال بين العمليات"))


def camera_malfunction_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  all_cams = ('roadCameraState')
  bad_cams = [s.replace('State', '') for s in all_cams if s in sm.data.keys() and not sm.all_checks([s, ])]
  return NormalPermanentAlert(_("عطل في الكاميرا"), ', '.join(bad_cams))


def calibration_invalid_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  rpy = sm['liveCalibration'].rpyCalib
  yaw = math.degrees(rpy[2] if len(rpy) == 3 else math.nan)
  pitch = math.degrees(rpy[1] if len(rpy) == 3 else math.nan)
  angles = f"Remount Device (Pitch: {pitch:.1f}°, Yaw: {yaw:.1f}°)"
  return NormalPermanentAlert(_("المعايرة غير صحيحة"), angles)


def overheat_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  cpu = max(sm['deviceState'].cpuTempC, default=0.)
  gpu = max(sm['deviceState'].gpuTempC, default=0.)
  temp = max((cpu, gpu, sm['deviceState'].memoryTempC))
  return NormalPermanentAlert(_("النظام ساخن"), f"{temp:.0f} °C")


def low_memory_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  return NormalPermanentAlert(_("الذاكرة ممتلئة"), f"{sm['deviceState'].memoryUsagePercent}% used")


def high_cpu_usage_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  x = max(sm['deviceState'].cpuUsagePercent, default=0.)
  return NormalPermanentAlert(_("استخدام عالي للمعالج"), _("%s%% مستخدم") % x)


def modeld_lagging_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  return NormalPermanentAlert(_("تأخر في نموذج القيادة"), f"{sm['modelV2'].frameDropPerc:.1f}% نزوب الإطارات")


def wrong_car_mode_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  text = _("قم بتفعيل نظام المثبت السرعة التكيفي للتشغيل")
  if CP.carName == "honda":
    text = _("قم بتفعيل نظام المثبت السرعة التكيفي للتشغيل")
  return NoEntryAlert(text)


def joystick_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  axes = sm['testJoystick'].axes
  gb, steer = list(axes)[:2] if len(axes) else (0., 0.)
  vals = f"Gas: {round(gb * 100.)}%, Steer: {round(steer * 100.)}%"
  return NormalPermanentAlert(_("وضع التحكم باليد"), vals)

def speed_limit_adjust_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  speedLimit = sm['longitudinalPlan'].speedLimit
  speed = round(speedLimit * (CV.MS_TO_KPH if metric else CV.MS_TO_MPH))
  message = _("Adjusting to %(speed)s %(unit)s") % ({"speed": speed, "unit": (_("km/h") if metric else _("mph"))})
  return Alert(
    message,
    "",
    AlertStatus.normal, AlertSize.small,
    Priority.LOW, VisualAlert.none, AudibleAlert.none, 4.)


EVENTS: Dict[int, Dict[str, Union[Alert, AlertCallbackType]]] = {
  # ********** events with no alerts **********

  EventName.stockFcw: {},

  # ********** events only containing alerts displayed in all states **********

  EventName.joystickDebug: {
    ET.WARNING: joystick_alert,
    ET.PERMANENT: NormalPermanentAlert(_("وضع التحكم باليد")),
  },

  EventName.controlsInitializing: {
    ET.NO_ENTRY: NoEntryAlert(_("جاري تهيئة النظام")),
  },

  EventName.startup: {
    ET.PERMANENT: StartupAlert(_("كن جاهزًا للتدخل في أي وقت"))
  },

  EventName.startupMaster: {
    ET.PERMANENT: startup_master_alert,
  },

  # Car is recognized, but marked as dashcam only
  EventName.startupNoControl: {
    ET.PERMANENT: StartupAlert(_("وضع الداشكام")),
  },

  # Car is not recognized
  EventName.startupNoCar: {
    ET.PERMANENT: StartupAlert(_("وضع الداشكام السيارة غير مدعومة")),
  },

  EventName.startupNoFw: {
    ET.PERMANENT: StartupAlert(_("لم يتم التعرف على السيارة"),
                               _("افحص منفذ الطاقة والاتصال بالسيارة"),
                               alert_status=AlertStatus.userPrompt),
  },

  EventName.dashcamMode: {
    ET.PERMANENT: NormalPermanentAlert(_("وضع الداشكام"),
                                       priority=Priority.LOWEST),
  },

  EventName.invalidLkasSetting: {
    ET.PERMANENT: NormalPermanentAlert(_("نظام LKAS مفعل"),
                                       _("افصل نظام LKAS لتفعيل القيادة الآلية")),
  },

  EventName.cruiseMismatch: {
    #ET.PERMANENT: ImmediateDisableAlert(_("openpilot failed to cancel cruise")),
  },

  # openpilot doesn't recognize the car. This switches openpilot into a
  # read-only mode. This can be solved by adding your fingerprint.
  # See https://github.com/commaai/openpilot/wiki/Fingerprinting for more information
  EventName.carUnrecognized: {
    ET.PERMANENT: NormalPermanentAlert(_("وضع الداشكام"),
                                       _("السيارة غير معروفة"),
                                       priority=Priority.LOWEST),
  },

  EventName.stockAeb: {
    ET.PERMANENT: Alert(
      _("فرمل!"),
      _("نظام الفرملة الطارئة الأساسي: خطر التصادم"),
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.fcw, AudibleAlert.none, 2.),
    ET.NO_ENTRY: NoEntryAlert(_("نظام الفرملة الطارئة الأساسي: خطر التصادم")),
  },

  EventName.fcw: {
    ET.PERMANENT: Alert(
      _("فرمل!"),
      _("خطر التصادم"),
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.fcw, AudibleAlert.warningSoft, 2.),
  },

  EventName.ldw: {
    ET.PERMANENT: Alert(
      _("تم اكتشاف خروج من المسار"),
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.ldw, AudibleAlert.prompt, 3.),
  },

  # ********** events only containing alerts that display while engaged **********

  # openpilot tries to learn certain parameters about your car by observing
  # how the car behaves to steering inputs from both human and openpilot driving.
  # This includes:
  # - steer ratio: gear ratio of the steering rack. Steering angle divided by tire angle
  # - tire stiffness: how much grip your tires have
  # - angle offset: most steering angle sensors are offset and measure a non zero angle when driving straight
  # This alert is thrown when any of these values exceed a sanity check. This can be caused by
  # bad alignment or bad sensor data. If this happens consistently consider creating an issue on GitHub
  EventName.vehicleModelInvalid: {
    ET.NO_ENTRY: NoEntryAlert(_("فشل في التعرف على معلمات السيارة")),
    ET.SOFT_DISABLE: soft_disable_alert(_("Vehicle Parameter Identification Failed")),
  },

  EventName.steerTempUnavailableSilent: {
    ET.WARNING: Alert(
      _("التوجيه غير متاح مؤقتًا"),
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.prompt, 1.8),
  },

  EventName.preDriverDistracted: {
    ET.WARNING: Alert(
      _("انتبه"),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1),
  },

  EventName.promptDriverDistracted: {
    ET.WARNING: Alert(
      _("انتبه"),
      _("السائق منشغل"),
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.promptDistracted, .1),
  },

  EventName.driverDistracted: {
    ET.WARNING: Alert(
      _("أوقف التشغيل فورًا"),
      _("السائق منشغل"),
      AlertStatus.critical, AlertSize.full,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.warningImmediate, .1),
  },

  EventName.preDriverUnresponsive: {
    ET.WARNING: Alert(
      _("المس المقود: لم يتم اكتشاف وجه"),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.none, .1, alert_rate=0.75),
  },

  EventName.promptDriverUnresponsive: {
    ET.WARNING: Alert(
      _("المس المقود"),
      _("السائق لا يستجيب"),
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.promptDistracted, .1),
  },

  EventName.driverUnresponsive: {
    ET.WARNING: Alert(
      _("أوقف التشغيل فورًا"),
      _("السائق لا يستجيب"),
      AlertStatus.critical, AlertSize.full,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.warningImmediate, .1),
  },

  EventName.manualRestart: {
    ET.WARNING: Alert(
      _("قم بالتحكم"),
      _("استمر في القيادة يدويًا"),
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .2),
  },

  EventName.resumeRequired: {
    ET.WARNING: Alert(
      _("اضغط على استئناف للخروج من الوقوف التام"),
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .2),
  },

  EventName.belowSteerSpeed: {
    ET.WARNING: below_steer_speed_alert,
  },

  EventName.preLaneChangeLeft: {
    ET.PERMANENT: Alert(
      _("قد إلى اليسار لبدء تغيير المسار عندما يكون ذلك آمنًا"),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1, alert_rate=0.75),
  },

  EventName.preLaneChangeRight: {
    ET.PERMANENT: Alert(
      _("قد إلى اليمين لبدء تغيير المسار عندما يكون ذلك آمنًا"),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1, alert_rate=0.75),
  },

  EventName.laneChangeBlocked: {
    ET.PERMANENT: Alert(
      _("تم اكتشاف سيارة في نقطة العمياء أو على حافة الطريق"),
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.prompt, .2),
  },

  EventName.laneChange: {
    ET.PERMANENT: Alert(
      _("يتم تغيير المسار"),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1),
  },

  EventName.steerSaturated: {
    ET.PERMANENT: Alert(
      _("قم بالتحكم"),
      _("الدوران يتجاوز حد التوجيه"),
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.promptRepeat, 1.),
  },

  # Thrown when the fan is driven at >50% but is not rotating
  EventName.fanMalfunction: {
    ET.PERMANENT: NormalPermanentAlert(_("عطل في المروحة"), _("عطل محتمل في المروحة")),
  },

  # Camera is not outputting frames
  EventName.cameraMalfunction: {
    ET.PERMANENT: camera_malfunction_alert,
    ET.SOFT_DISABLE: soft_disable_alert(_("عطل في الكاميرا")),
    ET.NO_ENTRY: NoEntryAlert(_("عطل في الكاميرا: أعد تشغيل الجهاز")),
  },
  # Camera framerate too low
  EventName.cameraFrameRate: {
    ET.PERMANENT: NormalPermanentAlert(_("إطارات الفيديو منخفظة"), _("أعد تشغيل الجهاز")),
    ET.SOFT_DISABLE: soft_disable_alert(_("إطارات الفيديو منخفظة")),
    ET.NO_ENTRY: NoEntryAlert(_("إطارات الفيديو منخفظة: أعد تشغيل الجهاز")),
  },

  # Unused
  EventName.gpsMalfunction: {
    ET.PERMANENT: NormalPermanentAlert(_("GPS عطل في"), _("عطل محتمل في تحديد مستشعر المواقع")),
  },

  # When the GPS position and localizer diverge the localizer is reset to the
  # current GPS position. This alert is thrown when the localizer is reset
  # more often than expected.
  EventName.localizerMalfunction: {
    # ET.PERMANENT: NormalPermanentAlert(_("Sensor Malfunction"), _("Hardware Malfunction")),
  },

  EventName.speedLimitActive: {
    ET.WARNING: Alert(
      "تم ضبط المثبت على حد السرعة",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 2.),
  },

  EventName.speedLimitValueChange: {
    ET.WARNING: speed_limit_adjust_alert,
  },

  # ********** events that affect controls state transitions **********

  EventName.pcmEnable: {
    ET.ENABLE: EngagementAlert(AudibleAlert.engage),
  },

  EventName.buttonEnable: {
    ET.ENABLE: EngagementAlert(AudibleAlert.engage),
  },

  EventName.pcmDisable: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
  },

  EventName.buttonCancel: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert("تم الضغط على الإلغاء"),
  },

  EventName.brakeHold: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert(_("دواسة الفرامل مفعلة")),
  },

  EventName.parkBrake: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert(_("تم تفعيل الجلنطd")),
  },

  EventName.pedalPressed: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert(_("تم الضغط على الدواسة"),
                              visual_alert=VisualAlert.brakePressed),
  },

  EventName.preEnableStandstill: {
    ET.PRE_ENABLE: Alert(
      _("أفلت دواسة الفرامل للتفعيل"),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .1, creation_delay=1.),
  },

  EventName.gasPressedOverride: {
    ET.OVERRIDE_LONGITUDINAL: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .1),
  },

  EventName.steerOverride: {
    ET.OVERRIDE_LATERAL: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .1),
  },

  EventName.wrongCarMode: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: wrong_car_mode_alert,
  },

  EventName.resumeBlocked: {
    ET.NO_ENTRY: NoEntryAlert("اضغط على تثبيت للتفعيل"),
  },

  EventName.wrongCruiseMode: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert(_("تم تعطيل مثبت السرعة")),
  },

  EventName.steerTempUnavailable: {
    ET.SOFT_DISABLE: soft_disable_alert(_("التوجيه غير متاح مؤقتًا")),
    ET.NO_ENTRY: NoEntryAlert(_("التوجيه غير متاح مؤقتًا")),
  },

  EventName.outOfSpace: {
    ET.PERMANENT: out_of_space_alert,
    ET.NO_ENTRY: NoEntryAlert(_("الذاكرة ممتلئة")),
  },

  EventName.belowEngageSpeed: {
    ET.NO_ENTRY: below_engage_speed_alert,
  },

  EventName.sensorDataInvalid: {
    ET.PERMANENT: Alert(
      _("بيانات الحساس غير صالحة"),
      _("تأكد من تثبيت الجهاز بأمان"),
      AlertStatus.normal, AlertSize.mid,
      Priority.LOWER, VisualAlert.none, AudibleAlert.none, .2, creation_delay=1.),
    ET.NO_ENTRY: NoEntryAlert(_("بيانات الحساس غير صالحة")),
    ET.SOFT_DISABLE: soft_disable_alert(_("بيانات الحساس غير صالحة")),
  },

  EventName.noGps: {
    ET.PERMANENT: no_gps_alert,
  },

  EventName.soundsUnavailable: {
    ET.PERMANENT: NormalPermanentAlert(_("Speaker not found"), _("Reboot your Device")),
    ET.NO_ENTRY: NoEntryAlert(_("Speaker not found")),
  },

  EventName.tooDistracted: {
    ET.NO_ENTRY: NoEntryAlert(_("مستوى انشغال السائق عالي")),
  },

  EventName.overheat: {
    ET.PERMANENT: overheat_alert,
    ET.SOFT_DISABLE: soft_disable_alert(_("النظام ساخن")),
    ET.NO_ENTRY: NoEntryAlert(_("النظام ساخن")),
  },

  EventName.wrongGear: {
    # ET.SOFT_DISABLE: user_soft_disable_alert(_("Gear not D")),
    ET.NO_ENTRY: NoEntryAlert(_("القير ليس على وضع القيادة")),
  },

  # This alert is thrown when the calibration angles are outside of the acceptable range.
  # For example if the device is pointed too much to the left or the right.
  # Usually this can only be solved by removing the mount from the windshield completely,
  # and attaching while making sure the device is pointed straight forward and is level.
  # See https://comma.ai/setup for more information
  EventName.calibrationInvalid: {
    ET.PERMANENT: calibration_invalid_alert,
    ET.SOFT_DISABLE: soft_disable_alert(_("المعايرة غير صالحة: أعد تثبيت الجهاز وأعايره مرة أخرى")),
    ET.NO_ENTRY: NoEntryAlert(_("المعايرة غير صحيحة: أعد تثبيت الجهاز وأعايره مرة أخرى")),
  },

  EventName.calibrationIncomplete: {
    ET.PERMANENT: calibration_incomplete_alert,
    ET.SOFT_DISABLE: soft_disable_alert(_("اكتشاف إعادة تثبيت الجهاز: إعادة المعايرة")),
    ET.NO_ENTRY: NoEntryAlert(_("اكتشاف إعادة التثبيت: إعادة المعايرة")),
  },

  EventName.doorOpen: {
    ET.SOFT_DISABLE: user_soft_disable_alert(_("الباب مفتوح")),
    ET.NO_ENTRY: NoEntryAlert(_("الباب مفتوح")),
  },

  EventName.seatbeltNotLatched: {
    ET.SOFT_DISABLE: user_soft_disable_alert(_("حزام الأمان غير مثبت")),
    ET.NO_ENTRY: NoEntryAlert(_("حزام الأمان غير مثبت")),
  },

  EventName.espDisabled: {
    ET.SOFT_DISABLE: soft_disable_alert(_("نظام التحكم الإلكتروني في الثبات معطل")),
    ET.NO_ENTRY: NoEntryAlert(_("نظام التحكم الإلكتروني في الثبات معطل")),
  },

  EventName.lowBattery: {
    ET.SOFT_DISABLE: soft_disable_alert(_("البطارية منخفضة")),
    ET.NO_ENTRY: NoEntryAlert(_("البطارية منخفضة")),
  },

  # Different openpilot services communicate between each other at a certain
  # interval. If communication does not follow the regular schedule this alert
  # is thrown. This can mean a service crashed, did not broadcast a message for
  # ten times the regular interval, or the average interval is more than 10% too high.
  EventName.commIssue: {
    ET.SOFT_DISABLE: soft_disable_alert(_("مشكلة في التواصل بين العمليات")),
    ET.NO_ENTRY: comm_issue_alert,
  },
  EventName.commIssueAvgFreq: {
    ET.SOFT_DISABLE: soft_disable_alert(_("معدل اتصال منخفض بين العمليات")),
    ET.NO_ENTRY: NoEntryAlert(_("معدل اتصال منخفض بين العمليات")),
  },

  EventName.controlsdLagging: {
    ET.SOFT_DISABLE: soft_disable_alert(_("تأخير في الضوابط")),
    ET.NO_ENTRY: NoEntryAlert(_("أخر في عملية التحكم: أعد تشغيل الجهاز الخاص بك")),
  },

  # Thrown when manager detects a service exited unexpectedly while driving
  EventName.processNotRunning: {
    ET.NO_ENTRY: process_not_running_alert,
    ET.SOFT_DISABLE: soft_disable_alert(_("العملية لا تعمل")),
  },

  EventName.radarFault: {
    ET.SOFT_DISABLE: soft_disable_alert(_("خطأ في الرادار: أعد تشغيل السيارة")),
    ET.NO_ENTRY: NoEntryAlert(_("خطأ في الرادار: أعد تشغيل السيارة")),
  },

  # Every frame from the camera should be processed by the model. If modeld
  # is not processing frames fast enough they have to be dropped. This alert is
  # thrown when over 20% of frames are dropped.
  EventName.modeldLagging: {
    ET.SOFT_DISABLE: soft_disable_alert(_("تأخر في نموذج القيادة")),
    ET.NO_ENTRY: NoEntryAlert(_("تأخر في نموذج القيادة")),
    ET.PERMANENT: modeld_lagging_alert,
  },

  # Besides predicting the path, lane lines and lead car data the model also
  # predicts the current velocity and rotation speed of the car. If the model is
  # very uncertain about the current velocity while the car is moving, this
  # usually means the model has trouble understanding the scene. This is used
  # as a heuristic to warn the driver.
  EventName.posenetInvalid: {
    ET.SOFT_DISABLE: soft_disable_alert(_("السرعة الحالية غير صالحة")),
    ET.NO_ENTRY: posenet_invalid_alert,
  },

  # When the localizer detects an acceleration of more than 40 m/s^2 (~4G) we
  # alert the driver the device might have fallen from the windshield.
  EventName.deviceFalling: {
    ET.SOFT_DISABLE: soft_disable_alert(_("الجهاز سقط من التثبيت")),
    ET.NO_ENTRY: NoEntryAlert(_("الجهاز سقط من التثبيت")),
  },

  EventName.lowMemory: {
    ET.SOFT_DISABLE: soft_disable_alert(_("الذاكرة منخفضة: أعد تشغيل الجهاز")),
    ET.PERMANENT: low_memory_alert,
    ET.NO_ENTRY: NoEntryAlert(_("الذاكرة منخفضة: أعد تشغيل الجهاز")),
  },

  EventName.highCpuUsage: {
    #ET.SOFT_DISABLE: soft_disable_alert(_("System Malfunction: Reboot Your Device")),
    #ET.PERMANENT: NormalPermanentAlert(_("System Malfunction"), _("Reboot your Device")),
    ET.NO_ENTRY: high_cpu_usage_alert,
  },

  EventName.accFaulted: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert(_("خطاء في نظام التحكم بالسرعة: أعد تشغيل السيارة")),
    ET.PERMANENT: NormalPermanentAlert(_("خطاء في نظام التحكم بالسرعة: أعد تشغيل السيارة للتفعيل")),
    ET.NO_ENTRY: NoEntryAlert(_("خطاء في نظام التحكم بالسرعة: أعد تشغيل السيارة")),
  },

  EventName.accFaultedTemp: {
    ET.NO_ENTRY: NoEntryAlert("عدم تطابق في التحكم"),
  },

  EventName.controlsMismatch: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert(_("عدم تطابق في التحكم")),
    ET.NO_ENTRY: NoEntryAlert(_("عدم تطابق في التحكم")),
  },

  EventName.roadCameraError: {
    ET.PERMANENT: NormalPermanentAlert(_("خطأ في جدول التحقق من الصحة للكاميرا - الطريق"),
                                       duration=1.,
                                       creation_delay=30.),
  },

  EventName.wideRoadCameraError: {
    ET.PERMANENT: NormalPermanentAlert(_("خطأ في جدول التحقق من الصحة للكاميرا - عدسة السمكة - الطريق"),
                                       duration=1.,
                                       creation_delay=30.),
  },

  EventName.driverCameraError: {
    ET.PERMANENT: NormalPermanentAlert(_("خطأ في جدول التحقق من الصحة للكاميرا - السائق"),
                                       duration=1.,
                                       creation_delay=30.),
  },

  # Sometimes the USB stack on the device can get into a bad state
  # causing the connection to the panda to be lost
  EventName.usbError: {
    ET.SOFT_DISABLE: soft_disable_alert(_("خطأ USB: أعد تشغيل جهازك")),
    ET.PERMANENT: NormalPermanentAlert(_("خطأ USB: أعد تشغيل جهازك"), ""),
    ET.NO_ENTRY: NoEntryAlert(_("خطأ USB: أعد تشغيل جهازك")),
  },

  # This alert can be thrown for the following reasons:
  # - No CAN data received at all
  # - CAN data is received, but some message are not received at the right frequency
  # If you're not writing a new car port, this is usually cause by faulty wiring
  EventName.canError: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert(_("خطاء في الكان")),
    ET.PERMANENT: Alert(
      _("خطأ CAN: قم بفحص الاتصالات"),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1., creation_delay=1.),
    ET.NO_ENTRY: NoEntryAlert(_("قاطع اتصال الشبكة الكان (CAN)")),
  },

  EventName.canBusMissing: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert(_("تم ثطع اتصال شبكة الكان (CAN)")),
    ET.PERMANENT: Alert(
      _("الاتصال بشبكة الكان قد تم قطعه: الكابل قد يكون به خلل"),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1., creation_delay=1.),
    ET.NO_ENTRY: NoEntryAlert(_("انقطع الاتصال بشبكة الكان: قم بفحص الاتصالات")),
  },

  EventName.steerUnavailable: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert(_("عطل في نظام مساعد الحفاظ على المسار (LKAS): أعد تشغيل السيارة")),
    ET.PERMANENT: NormalPermanentAlert(_("عطل في نظام مساعد الحفاظ على المسار (LKAS): قم بإعادة تشغيل السيارة للتفعيل")),
    ET.NO_ENTRY: NoEntryAlert(_("خلل في نظام الحفاظ على المسار (LKAS): أعد تشغيل السيارة")),
  },

  EventName.brakeUnavailable: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert(_("عطل في نظام المثبت السرعة: أعد تشغيل السيارة")),
    ET.PERMANENT: NormalPermanentAlert(_("عطل في نظام المثبت السرعة: أعد تشغيل السيارة للتفعيل")),
    ET.NO_ENTRY: NoEntryAlert(_("عطل في نظام المثبت السرعة: أعد تشغيل السيارة")),
  },

  EventName.reverseGear: {
    ET.PERMANENT: Alert(
      _("الرجوع\nللخلف"),
      "",
      AlertStatus.normal, AlertSize.full,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .2, creation_delay=0.5),
    ET.USER_DISABLE: ImmediateDisableAlert(_("الرجوع للخلف")),
    ET.NO_ENTRY: NoEntryAlert(_("الرجوع للخلف")),
  },

  # On cars that use stock ACC the car can decide to cancel ACC for various reasons.
  # When this happens we can no long control the car so the user needs to be warned immediately.
  EventName.cruiseDisabled: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert(_("المثبت مطفاء")),
  },

  # For planning the trajectory Model Predictive Control (MPC) is used. This is
  # an optimization algorithm that is not guaranteed to find a feasible solution.
  # If no solution is found or the solution has a very high cost this alert is thrown.
  EventName.plannerError: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert(_("خطاء في تخطيط المسار")),
    ET.NO_ENTRY: NoEntryAlert(_("خطاء في تخطيط المسار")),
  },

  # When the relay in the harness box opens the CAN bus between the LKAS camera
  # and the rest of the car is separated. When messages from the LKAS camera
  # are received on the car side this usually means the relay hasn't opened correctly
  # and this alert is thrown.
  EventName.relayMalfunction: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert(_("عطل في ريلاي الظفيرة")),
    ET.PERMANENT: NormalPermanentAlert(_("عطل في ريلاي الظفيرة"), _("افحص الاجزاء")),
    ET.NO_ENTRY: NoEntryAlert(_("عطل في ريلاي الظفيرة")),
  },

  EventName.speedTooLow: {
    ET.IMMEDIATE_DISABLE: Alert(
      _("تم الغاء القائد الآلي"),
      _("السرعة منخفضة جداً"),
      AlertStatus.normal, AlertSize.mid,
      Priority.HIGH, VisualAlert.none, AudibleAlert.disengage, 3.),
  },

  # When the car is driving faster than most cars in the training data, the model outputs can be unpredictable.
  EventName.speedTooHigh: {
    ET.WARNING: Alert(
      _("السرعة مرتفعة جداً"),
      _("المودل لا يعمل بشكل على هذه السرعة"),
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.promptRepeat, 4.),
    ET.NO_ENTRY: NoEntryAlert(_("قم بخفض السرعة للتفعيل")),
  },

  EventName.lowSpeedLockout: {
    ET.PERMANENT: NormalPermanentAlert(_("خطاء في مثبت السرعة: أعد تشغيل السيارة")),
    ET.NO_ENTRY: NoEntryAlert(_("خطاء في مثبت السرعة: أعد تشغيل السيارة")),
  },

  EventName.lkasDisabled: {
    ET.PERMANENT: NormalPermanentAlert(_("تعطيل نظام مساعد الحفاظ على المسار (LKAS): قم بتمكين النظام للتفعيل")),
    ET.NO_ENTRY: NoEntryAlert(_("نظام مساعدة الحفاظ على المسار (LKAS) معطل")),
  },

  # dp - use for atl alert
  EventName.communityFeatureDisallowedDEPRECATED: {
    ET.OVERRIDE_LATERAL: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.MID, VisualAlert.none,
      AudibleAlert.disengage, .2),
  },

  # dp - use for manual lane change
  EventName.manualSteeringRequiredBlinkersOn: {
    ET.PERMANENT: Alert(
      _("التوجيه مطلوب: الإشارة مفعلة"),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .0, alert_rate=0.25),
  },
}