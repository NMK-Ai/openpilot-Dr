from collections import defaultdict, namedtuple
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Union

from cereal import car
from panda.python import uds
from opendbc.can.can_define import CANDefine
from selfdrive.car import dbc_dict
from selfdrive.car.docs_definitions import CarFootnote, CarInfo, Column, Harness
from selfdrive.car.fw_query_definitions import FwQueryConfig, Request, p16

Ecu = car.CarParams.Ecu
NetworkLocation = car.CarParams.NetworkLocation
TransmissionType = car.CarParams.TransmissionType
GearShifter = car.CarState.GearShifter
Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])


class CarControllerParams:
  STEER_STEP = 2                          # تردد رسالة HCA_01/HCA_1 هو 50 هرتز.
  ACC_CONTROL_STEP = 2                    # تردد نظام ACC_06/ACC_07/ACC هو 50 هرتز.

  ACCEL_MAX = 2.0                         # التسارع الأقصى هو 2.0 م/ث.
  ACCEL_MIN = -3.5                        # التباطؤ الأقصى هو 3.5 م/ث.

  def __init__(self, CP):
  # الحدود الموثقة للانعطاف الجانبي: 3.00 نيوتن متر كحد أقصى، معدل التغير 5.00 نيوتن متر/ثانية.
  # الحدود القصوى لنظام MQB ونظام PQ مشتركة، لكن معدل التغير مختلف بناءً على متطلبات الأمان
  # التي تعتمد على اختبارات التسارع الجانبي.
    self.STEER_MAX = 300                  # Max heading control assist torque 3.00 Nm
    self.STEER_DRIVER_MULTIPLIER = 3      # weight driver torque heavily
    self.STEER_DRIVER_FACTOR = 1          # from dbc

    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])

    if CP.carFingerprint in PQ_CARS:
      self.LDW_STEP = 5                   # تردد رسالة LDW_1 هو 20 هرتز.
      self.ACC_HUD_STEP = 4               # تردد رسالة ACC_GRA_Anzeige هو 25 هرتز.
      self.STEER_DRIVER_ALLOWANCE = 80    # عتبة تدخل السائق هي 0.8 نيوتن متر.
      self.STEER_DELTA_UP = 6             # أقصى قيمة لنظام HCA تتحقق خلال 1.00 ثانية (STEER_MAX / (50Hz * 1.00)).
      self.STEER_DELTA_DOWN = 10          # أدنى قيمة لنظام HCA تتحقق خلال 0.60 ثانية (STEER_MAX / (50Hz * 0.60)).

      if CP.transmissionType == TransmissionType.automatic:
        self.shifter_values = can_define.dv["Getriebe_1"]["Waehlhebelposition__Getriebe_1_"]
      self.hca_status_values = can_define.dv["Lenkhilfe_2"]["LH2_Sta_HCA"]

      self.BUTTONS = [
        Button(car.CarState.ButtonEvent.Type.setCruise, "GRA_Neu", "GRA_Neu_Setzen", [1]),
        Button(car.CarState.ButtonEvent.Type.resumeCruise, "GRA_Neu", "GRA_Recall", [1]),
        Button(car.CarState.ButtonEvent.Type.accelCruise, "GRA_Neu", "GRA_Up_kurz", [1]),
        Button(car.CarState.ButtonEvent.Type.decelCruise, "GRA_Neu", "GRA_Down_kurz", [1]),
        Button(car.CarState.ButtonEvent.Type.cancel, "GRA_Neu", "GRA_Abbrechen", [1]),
        Button(car.CarState.ButtonEvent.Type.gapAdjustCruise, "GRA_Neu", "GRA_Zeitluecke", [1]),
      ]

      self.LDW_MESSAGES = {
        "none": 0,  # لا يوجد شيء لعرضه.
        "laneAssistUnavail": 1,  # "المساعدة في الحفاظ على المسار غير متوفرة حالياً."
        "laneAssistUnavailSysError": 2,  # "خطأ في نظام المساعدة في الحفاظ على المسار."
        "laneAssistUnavailNoSensorView": 3,  # "المساعدة في الحفاظ على المسار غير متوفرة. لا توجد رؤية للمستشعر."
        "laneAssistTakeOver": 4,  # "المساعدة في الحفاظ على المسار: يرجى تولي عملية التوجيه."
        "laneAssistDeactivTrailer": 5,  # "المساعدة في الحفاظ على المسار: لا توجد وظيفة مع المقطورة."
      }

    else:
      self.LDW_STEP = 10                  # تردد رسالة LDW_02 هو 10 هرتز.
      self.ACC_HUD_STEP = 6               # تردد رسالة ACC_02 هو 16 هرتز
      self.STEER_DRIVER_ALLOWANCE = 80    # عتبة تدخل السائق هي 0.8 نيوتن متر
      self.STEER_DELTA_UP = 4             # أقصى قيمة لنظام HCA تتحقق في 1.50 ثانية (STEER_MAX / (50Hz * 1.50))
      self.STEER_DELTA_DOWN = 10          # أدنى قيمة لنظام HCA تتحقق في 0.60 ثانية (STEER_MAX / (50Hz * 0.60))

      if CP.transmissionType == TransmissionType.automatic:
        self.shifter_values = can_define.dv["Getriebe_11"]["GE_Fahrstufe"]
      elif CP.transmissionType == TransmissionType.direct:
        self.shifter_values = can_define.dv["EV_Gearshift"]["GearPosition"]
      self.hca_status_values = can_define.dv["LH_EPS_03"]["EPS_HCA_Status"]

      self.BUTTONS = [
        Button(car.CarState.ButtonEvent.Type.setCruise, "GRA_ACC_01", "GRA_Tip_Setzen", [1]),
        Button(car.CarState.ButtonEvent.Type.resumeCruise, "GRA_ACC_01", "GRA_Tip_Wiederaufnahme", [1]),
        Button(car.CarState.ButtonEvent.Type.accelCruise, "GRA_ACC_01", "GRA_Tip_Hoch", [1]),
        Button(car.CarState.ButtonEvent.Type.decelCruise, "GRA_ACC_01", "GRA_Tip_Runter", [1]),
        Button(car.CarState.ButtonEvent.Type.cancel, "GRA_ACC_01", "GRA_Abbrechen", [1]),
        Button(car.CarState.ButtonEvent.Type.gapAdjustCruise, "GRA_ACC_01", "GRA_Verstellung_Zeitluecke", [1]),
      ]

      self.LDW_MESSAGES = {
        "none": 0,                            # لا يوجد شيء لعرضه
        "laneAssistUnavailChime": 1,          # "المساعدة في الحفاظ على المسار غير متوفرة حالياً." مع صوت تنبيه
        "laneAssistUnavailNoSensorChime": 3,  # "المساعدة في الحفاظ على المسار غير متوفرة. لا توجد رؤية للمستشعر." مع صوت تنبيه
        "laneAssistTakeOverUrgent": 4,        # "المساعدة في الحفاظ على المسار: يرجى تولي عملية التوجيه." مع تنبيه عاجل
        "emergencyAssistUrgent": 6,           # "المساعدة الطارئة: يرجى تولي عملية التوجيه." مع تنبيه عاجل
        "laneAssistTakeOverChime": 7,         # "المساعدة في الحفاظ على المسار: يرجى تولي عملية التوجيه." مع صوت تنبيه
        "laneAssistTakeOver": 8,              # "المساعدة في الحفاظ على المسار: يرجى تولي عملية التوجيه." بصمت
        "emergencyAssistChangingLanes": 9,    # "المساعدة الطارئة: تغيير المسار..." مع تنبيه عاجل
        "laneAssistDeactivated": 10,          # "المساعدة في الحفاظ على المسار تم تعطيلها." بصمت مع أيقونة مستمرة بعد ذلك
      }


class CANBUS:
  pt = 0
  cam = 2


# Check the 7th and 8th characters of the VIN before adding a new CAR. If the
# chassis code is already listed below, don't add a new CAR, just add to the
# FW_VERSIONS for that existing CAR.
# Exception: SEAT Leon and SEAT Ateca share a chassis code

class CAR:
  ARTEON_MK1 = "فولكس فاجن أرتيون الجيل الأول"         # هيكل AN، الجيل الأول من فولكس فاجن أرتيون ومشتقاتها
  ATLAS_MK1 = "فولكس فاجن أطلس الجيل الأول"            # هيكل CA، الجيل الأول من فولكس فاجن أطلس وأطلس كروس سبورت
  CRAFTER_MK2 = "فولكس فاجن كرافتير الجيل الثاني"    # هيكل SY/SZ، الجيل الثاني من فولكس فاجن كرافتير، فولكس فاجن جراند كاليفورنيا، MAN TGE
  GOLF_MK7 = "فولكس فاجن جولف الجيل السابع"          # هيكل 5G/AU/BA/BE، الجيل السابع من فولكس فاجن جولف ومشتقاتها
  JETTA_MK7 = "فولكس فاجن جيتا الجيل السابع"         # هيكل BU، الجيل السابع من فولكس فاجن جيتا
  PASSAT_MK8 = "فولكس فاجن باسات الجيل الثامن"       # هيكل 3G، الجيل الثامن من فولكس فاجن باسات ومشتقاتها
  PASSAT_NMS = "فولكس فاجن باسات NMS"                 # هيكل A3، باسات NMS لأمريكا الشمالية/الصين/الشرق الأوسط، يشمل الفيس ليفت
  POLO_MK6 = "فولكس فاجن بولو الجيل السادس"          # هيكل AW، الجيل السادس من فولكس فاجن بولو
  SHARAN_MK2 = "فولكس فاجن شاران الجيل الثاني"       # هيكل 7N، الجيل الثاني من فولكس فاجن شاران وسيات ألهامبرا
  TAOS_MK1 = "فولكس فاجن تاوس الجيل الأول"            # هيكل B2، الجيل الأول من فولكس فاجن تاوس وثارو
  TCROSS_MK1 = "فولكس فاجن تي-كروس الجيل الأول"       # هيكل C1، الجيل الأول من فولكس فاجن تي-كروس المتغيرات القصيرة والطويلة
  TIGUAN_MK2 = "فولكس فاجن تيغوان الجيل الثاني"      # هيكل AD/BW، الجيل الثاني من فولكس فاجن تيغوان ومشتقاتها
  TOURAN_MK2 = "فولكس فاجن توران الجيل الثاني"       # هيكل 1T، الجيل الثاني من فولكس فاجن توران ومشتقاتها
  TRANSPORTER_T61 = "فولكس فاجن ترانسبورتر T6.1"      # هيكل 7H/7L، الجيل المعاد تصميمه من فولكس فاجن ترانسبورتر/مولتيفان/كارافيل/كاليفورنيا
  TROC_MK1 = "فولكس فاجن تي-روك الجيل الأول"          # هيكل A1، الجيل الأول من فولكس فاجن تي-روك ومشتقاتها
  AUDI_A3_MK3 = "أودي A3 الجيل الثالث"                # هيكل 8V/FF، الجيل الثالث من أودي A3 ومشتقاتها
  AUDI_Q2_MK1 = "أودي Q2 الجيل الأول"                  # هيكل GA، الجيل الأول من أودي Q2 (بقية العالم) وQ2L (للصين فقط)
  AUDI_Q3_MK2 = "أودي Q3 الجيل الثاني"               # هيكل 8U/F3/FS، الجيل الثاني من أودي Q3 ومشتقاتها
  SEAT_ATECA_MK1 = "سيات أتيكا الجيل الأول"           # هيكل 5F، الجيل الأول من سيات أتيكا وكوبرا أتيكا
  SEAT_LEON_MK3 = "سيات ليون الجيل الثالث"           # هيكل 5F، الجيل الثالث من سيات ليون ومشتقاتها
  SKODA_FABIA_MK4 = "سكودا فابيا الجيل الرابع"       # هيكل PJ، الجيل الرابع من سكودا فابيا
  SKODA_KAMIQ_MK1 = "سكودا كاميك الجيل الأول"         # هيكل NW، الجيل الأول من سكودا كاميك
  SKODA_KAROQ_MK1 = "سكودا كاروك الجيل الأول"         # هيكل NU، الجيل الأول من سكودا كاروك
  SKODA_KODIAQ_MK1 = "سكودا كودياك الجيل الأول"       # هيكل NS، الجيل الأول من سكودا كودياك
  SKODA_SCALA_MK1 = "سكودا سكالا الجيل الأول"          # هيكل NW، الجيل الأول من سكودا سكالا وسكودا كاميك
  SKODA_SUPERB_MK3 = "سكودا سوبيرب الجيل الثالث"     # هيكل 3V/NP، الجيل الثالث من سكودا سوبيرب ومشتقاتها
  SKODA_OCTAVIA_MK3 = "سكودا أوكتافيا الجيل الثالث"  # هيكل NE، الجيل الثالث من سكودا أوكتافيا ومشتقاتها


# السيارات المبنية على منصات PQ35/PQ46/NMS والتي تستخدم رسائل CAN بنمط PQ (الإعداد الافتراضي هو MQB)
PQ_CARS = {CAR.PASSAT_NMS, CAR.SHARAN_MK2}

# السيارات التي تستفيد من إرسال مستمر لأمر الاستئناف أثناء التوقف، مما يطور FtS الأصلي إلى SnG
# يبدو أن ذلك يشمل سيارات MQB-A0 وعائلة Transporter
# ملاحظة: حاول تحديد ذلك من خلال برنامج ABS الثابت بدلاً من ذلك
STANDING_RESUME_SPAM_CARS = {CAR.POLO_MK6, CAR.TCROSS_MK1, CAR.TROC_MK1, CAR.SKODA_KAMIQ_MK1,
                             CAR.SKODA_SCALA_MK1, CAR.TRANSPORTER_T61}


DBC: Dict[str, Dict[str, str]] = defaultdict(lambda: dbc_dict("vw_mqb_2010", None))
for car_type in PQ_CARS:
  DBC[car_type] = dbc_dict("vw_golf_mk4", None)


class Footnote(Enum):
  KAMIQ = CarFootnote(
    "Not including the China market Kamiq, which is based on the (currently) unsupported PQ34 platform.",
    Column.MODEL)
  PASSAT = CarFootnote(
    "Refers only to the MQB-based European B8 Passat, not the NMS Passat in the USA/China/Mideast markets.",
    Column.MODEL)
  VW_EXP_LONG = CarFootnote (
    "Only available for vehicles using a gateway (J533) harness. At this time, vehicles using a camera harness " +
    "are limited to using stock ACC.",
    Column.LONGITUDINAL)
  VW_MQB_A0 = CarFootnote(
    "Model-years 2022 and beyond may have a combined CAN gateway and BCM, which is supported by openpilot " +
    "in software, but doesn't yet have a harness available from the comma store.",
    Column.HARNESS)


@dataclass
class VWCarInfo(CarInfo):
  package: str = "Adaptive Cruise Control (ACC) & Lane Assist"
  harness: Enum = Harness.j533

  def init_make(self, CP: car.CarParams):
    self.footnotes.insert(0, Footnote.VW_EXP_LONG)


CAR_INFO: Dict[str, Union[VWCarInfo, List[VWCarInfo]]] = {
  CAR.ARTEON_MK1: [
    VWCarInfo("Volkswagen Arteon 2018-22", video_link="https://youtu.be/FAomFKPFlDA"),
    VWCarInfo("Volkswagen Arteon R 2020-22", video_link="https://youtu.be/FAomFKPFlDA"),
    VWCarInfo("Volkswagen Arteon eHybrid 2020-22", video_link="https://youtu.be/FAomFKPFlDA"),
    VWCarInfo("Volkswagen CC 2018-22", video_link="https://youtu.be/FAomFKPFlDA"),
  ],
  CAR.ATLAS_MK1: [
    VWCarInfo("Volkswagen Atlas 2018-23"),
    VWCarInfo("Volkswagen Atlas Cross Sport 2021-22"),
    VWCarInfo("Volkswagen Teramont 2018-22"),
    VWCarInfo("Volkswagen Teramont Cross Sport 2021-22"),
    VWCarInfo("Volkswagen Teramont X 2021-22"),
  ],
  CAR.CRAFTER_MK2: [
    VWCarInfo("Volkswagen Crafter 2017-23", video_link="https://youtu.be/4100gLeabmo"),
    VWCarInfo("Volkswagen e-Crafter 2018-23", video_link="https://youtu.be/4100gLeabmo"),
    VWCarInfo("Volkswagen Grand California 2019-23", video_link="https://youtu.be/4100gLeabmo"),
    VWCarInfo("MAN TGE 2017-23", video_link="https://youtu.be/4100gLeabmo"),
    VWCarInfo("MAN eTGE 2020-23", video_link="https://youtu.be/4100gLeabmo"),
  ],
  CAR.GOLF_MK7: [
    VWCarInfo("Volkswagen e-Golf 2014-20"),
    VWCarInfo("Volkswagen Golf 2015-20"),
    VWCarInfo("Volkswagen Golf Alltrack 2015-19"),
    VWCarInfo("Volkswagen Golf GTD 2015-20"),
    VWCarInfo("Volkswagen Golf GTE 2015-20"),
    VWCarInfo("Volkswagen Golf GTI 2015-21"),
    VWCarInfo("Volkswagen Golf R 2015-19"),
    VWCarInfo("Volkswagen Golf SportsVan 2015-20"),
  ],
  CAR.JETTA_MK7: [
    VWCarInfo("Volkswagen Jetta 2018-22"),
    VWCarInfo("Volkswagen Jetta GLI 2021-22"),
  ],
  CAR.PASSAT_MK8: [
    VWCarInfo("Volkswagen Passat 2015-22", footnotes=[Footnote.PASSAT]),
    VWCarInfo("Volkswagen Passat Alltrack 2015-22"),
    VWCarInfo("Volkswagen Passat GTE 2015-22"),
  ],
  CAR.PASSAT_NMS: VWCarInfo("Volkswagen Passat NMS 2017-22"),
  CAR.POLO_MK6: [
    VWCarInfo("Volkswagen Polo 2020-22", footnotes=[Footnote.VW_MQB_A0]),
    VWCarInfo("Volkswagen Polo GTI 2020-22", footnotes=[Footnote.VW_MQB_A0]),
  ],
  CAR.SHARAN_MK2: [
    VWCarInfo("Volkswagen Sharan 2018-22"),
    VWCarInfo("SEAT Alhambra 2018-20"),
  ],
  CAR.TAOS_MK1: VWCarInfo("Volkswagen Taos 2022"),
  CAR.TCROSS_MK1: VWCarInfo("Volkswagen T-Cross 2021", footnotes=[Footnote.VW_MQB_A0]),
  CAR.TIGUAN_MK2: VWCarInfo("Volkswagen Tiguan 2018-23"),
  CAR.TOURAN_MK2: VWCarInfo("Volkswagen Touran 2017"),
  CAR.TRANSPORTER_T61: [
    VWCarInfo("Volkswagen Caravelle 2020"),
    VWCarInfo("Volkswagen California 2021"),
  ],
  CAR.TROC_MK1: VWCarInfo("Volkswagen T-Roc 2021", footnotes=[Footnote.VW_MQB_A0]),
  CAR.AUDI_A3_MK3: [
    VWCarInfo("Audi A3 2014-19"),
    VWCarInfo("Audi A3 Sportback e-tron 2017-18"),
    VWCarInfo("Audi RS3 2018"),
    VWCarInfo("Audi S3 2015-17"),
  ],
  CAR.AUDI_Q2_MK1: VWCarInfo("Audi Q2 2018"),
  CAR.AUDI_Q3_MK2: VWCarInfo("Audi Q3 2019-23"),
  CAR.SEAT_ATECA_MK1: VWCarInfo("SEAT Ateca 2018"),
  CAR.SEAT_LEON_MK3: VWCarInfo("SEAT Leon 2014-20"),
  CAR.SKODA_FABIA_MK4: VWCarInfo("Škoda Fabia 2022-23", footnotes=[Footnote.VW_MQB_A0]),
  CAR.SKODA_KAMIQ_MK1: VWCarInfo("Škoda Kamiq 2021", footnotes=[Footnote.VW_MQB_A0, Footnote.KAMIQ]),
  CAR.SKODA_KAROQ_MK1: VWCarInfo("Škoda Karoq 2019-21"),
  CAR.SKODA_KODIAQ_MK1: VWCarInfo("Škoda Kodiaq 2017-23"),
  CAR.SKODA_SCALA_MK1: VWCarInfo("Škoda Scala 2020", footnotes=[Footnote.VW_MQB_A0]),
  CAR.SKODA_SUPERB_MK3: VWCarInfo("Škoda Superb 2015-22"),
  CAR.SKODA_OCTAVIA_MK3: [
    VWCarInfo("Škoda Octavia 2015, 2018-19"),
    VWCarInfo("Škoda Octavia RS 2016"),
  ],
}


# يجب أن تعود جميع السيارات المدعومة بمعلومات FW من المحرك، ونظام SRS، ونظام EPS، ورادار FWD.
# السيارات ذات ناقل الحركة اليدوي لن تعيد البرنامج الثابت لناقل الحركة، ولكن جميع السيارات الأخرى ستقوم بذلك.
#
# استعلام الرقم التسلسلي للبرنامج 0xF187 يجب أن يعود بالشكل N[NX][NX] NNN NNN [X[X]]،
# حيث N = رقم، X = حرف، والحرفان الأخيران اختياريان. 
# بعض معدلي الأداء يعبثون بهذا الحقل (على سبيل المثال: 8V0 9C0 BB0 1 من COBB/EQT).
# أرقام البرامج الثابتة المعدلة لوحدة التحكم الإلكترونية (ECU) غير صالحة لتحديد هوية السيارة وفحوصات التوافق.
# حاول إصلاحها من قبل المعدل قبل تضمينها في openpilot.

VOLKSWAGEN_VERSION_REQUEST_MULTI = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_SPARE_PART_NUMBER) + \
  p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_ECU_SOFTWARE_VERSION_NUMBER) + \
  p16(uds.DATA_IDENTIFIER_TYPE.APPLICATION_DATA_IDENTIFICATION)
VOLKSWAGEN_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40])

VOLKSWAGEN_RX_OFFSET = 0x6a

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [VOLKSWAGEN_VERSION_REQUEST_MULTI],
      [VOLKSWAGEN_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.srs, Ecu.eps, Ecu.fwdRadar],
      rx_offset=VOLKSWAGEN_RX_OFFSET,
    ),
    Request(
      [VOLKSWAGEN_VERSION_REQUEST_MULTI],
      [VOLKSWAGEN_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine, Ecu.transmission],
    ),
  ],
)

FW_VERSIONS = {
  CAR.ARTEON_MK1: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x873G0906259F \xf1\x890004',
      b'\xf1\x873G0906259N \xf1\x890004',
      b'\xf1\x873G0906259P \xf1\x890001',
      b'\xf1\x875NA907115H \xf1\x890002',
      b'\xf1\x873G0906259G \xf1\x890004',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x8709G927158L \xf1\x893611',
      b'\xf1\x870GC300011L \xf1\x891401',
      b'\xf1\x870GC300014M \xf1\x892802',
      b'\xf1\x870GC300040P \xf1\x891401',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x873Q0959655BK\xf1\x890703\xf1\x82\x0e1616001613121157161111572900',
      b'\xf1\x873Q0959655BK\xf1\x890703\xf1\x82\x0e1616001613121177161113772900',
      b'\xf1\x873Q0959655DL\xf1\x890732\xf1\x82\0161812141812171105141123052J00',
      b'\xf1\x873Q0959655CK\xf1\x890711\xf1\x82\x0e1712141712141105121122052900',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x873Q0909144K \xf1\x895072\xf1\x82\x0571B41815A1',
      b'\xf1\x873Q0909144L \xf1\x895081\xf1\x82\x0571B00817A1',
      b'\xf1\x875Q0910143C \xf1\x892211\xf1\x82\x0567B0020800',
      b'\xf1\x875WA907145M \xf1\x891051\xf1\x82\x002MB4092M7N',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x872Q0907572AA\xf1\x890396',
      b'\xf1\x872Q0907572T \xf1\x890383',
      b'\xf1\x875Q0907572J \xf1\x890654',
      b'\xf1\x875Q0907572R \xf1\x890771',
    ],
  },
  CAR.ATLAS_MK1: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8703H906026AA\xf1\x899970',
      b'\xf1\x8703H906026AJ\xf1\x890638',
      b'\xf1\x8703H906026AJ\xf1\x891017',
      b'\xf1\x8703H906026AT\xf1\x891922',
      b'\xf1\x8703H906026BC\xf1\x892664',
      b'\xf1\x8703H906026F \xf1\x896696',
      b'\xf1\x8703H906026F \xf1\x899970',
      b'\xf1\x8703H906026J \xf1\x896026',
      b'\xf1\x8703H906026J \xf1\x899971',
      b'\xf1\x8703H906026S \xf1\x896693',
      b'\xf1\x8703H906026S \xf1\x899970',
      b'\xf1\x873CN906259  \xf1\x890005',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x8709G927158A \xf1\x893387',
      b'\xf1\x8709G927158DR\xf1\x893536',
      b'\xf1\x8709G927158DR\xf1\x893742',
      b'\xf1\x8709G927158F \xf1\x893489',
      b'\xf1\x8709G927158FT\xf1\x893835',
      b'\xf1\x8709G927158GL\xf1\x893939',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x873Q0959655BC\xf1\x890503\xf1\x82\0161914151912001103111122031200',
      b'\xf1\x873Q0959655BN\xf1\x890713\xf1\x82\0162214152212001105141122052900',
      b'\xf1\x873Q0959655DB\xf1\x890720\xf1\x82\0162214152212001105141122052900',
      b'\xf1\x873Q0959655DM\xf1\x890732\xf1\x82\x0e1114151112001105161122052J00',
      b'\xf1\x873Q0959655DM\xf1\x890732\xf1\x82\x0e1115151112001105171122052J00',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x873QF909144B \xf1\x891582\xf1\x82\00571B60924A1',
      b'\xf1\x873QF909144B \xf1\x891582\xf1\x82\x0571B6G920A1',
      b'\xf1\x875Q0909143P \xf1\x892051\xf1\x820528B6080105',
      b'\xf1\x875Q0909143P \xf1\x892051\xf1\x820528B6090105',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x872Q0907572AA\xf1\x890396',
      b'\xf1\x872Q0907572R \xf1\x890372',
      b'\xf1\x872Q0907572T \xf1\x890383',
      b'\xf1\x875Q0907572H \xf1\x890620',
      b'\xf1\x875Q0907572J \xf1\x890654',
      b'\xf1\x875Q0907572P \xf1\x890682',
    ],
  },
  CAR.CRAFTER_MK2: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8704L906056EK\xf1\x896391',
    ],
    # السيارة الوحيدة التي تم دمجها حالياً وتحتوي على ناقل حركة يدوي
    # (Ecu.transmission, 0x7e1, None): [
    # ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x873Q0959655BG\xf1\x890703\xf1\x82\x0e16120016130012051G1313052900',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x872N0909143E \xf1\x897021\xf1\x82\x05163AZ306A2',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x872Q0907572M \xf1\x890233',
    ],
  },
  CAR.GOLF_MK7: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8704E906016A \xf1\x897697',
      b'\xf1\x8704E906016AD\xf1\x895758',
      b'\xf1\x8704E906016CE\xf1\x899096',
      b'\xf1\x8704E906023AG\xf1\x891726',
      b'\xf1\x8704E906023BN\xf1\x894518',
      b'\xf1\x8704E906024K \xf1\x896811',
      b'\xf1\x8704E906027GR\xf1\x892394',
      b'\xf1\x8704E906027HD\xf1\x892603',
      b'\xf1\x8704E906027HD\xf1\x893742',
      b'\xf1\x8704E906027MA\xf1\x894958',
      b'\xf1\x8704L906021DT\xf1\x895520',
      b'\xf1\x8704L906021DT\xf1\x898127',
      b'\xf1\x8704L906021N \xf1\x895518',
      b'\xf1\x8704L906026BN\xf1\x891197',
      b'\xf1\x8704L906026BP\xf1\x897608',
      b'\xf1\x8704L906026NF\xf1\x899528',
      b'\xf1\x8704L906056CL\xf1\x893823',
      b'\xf1\x8704L906056CR\xf1\x895813',
      b'\xf1\x8704L906056HE\xf1\x893758',
      b'\xf1\x8704L906056HN\xf1\x896590',
      b'\xf1\x8704L906056HT\xf1\x896591',
      b'\xf1\x870EA906016A \xf1\x898343',
      b'\xf1\x870EA906016E \xf1\x894219',
      b'\xf1\x870EA906016F \xf1\x894238',
      b'\xf1\x870EA906016F \xf1\x895002',
      b'\xf1\x870EA906016Q \xf1\x895993',
      b'\xf1\x870EA906016S \xf1\x897207',
      b'\xf1\x875G0906259  \xf1\x890007',
      b'\xf1\x875G0906259D \xf1\x890002',
      b'\xf1\x875G0906259J \xf1\x890002',
      b'\xf1\x875G0906259L \xf1\x890002',
      b'\xf1\x875G0906259N \xf1\x890003',
      b'\xf1\x875G0906259Q \xf1\x890002',
      b'\xf1\x875G0906259Q \xf1\x892313',
      b'\xf1\x875G0906259T \xf1\x890003',
      b'\xf1\x878V0906259H \xf1\x890002',
      b'\xf1\x878V0906259J \xf1\x890003',
      b'\xf1\x878V0906259K \xf1\x890001',
      b'\xf1\x878V0906259P \xf1\x890001',
      b'\xf1\x878V0906259Q \xf1\x890002',
      b'\xf1\x878V0906264F \xf1\x890003',
      b'\xf1\x878V0906264L \xf1\x890002',
      b'\xf1\x878V0906264M \xf1\x890001',
      b'\xf1\x878V09C0BB01 \xf1\x890001',
      b'\xf1\x8704E906024K \xf1\x899970',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x8709G927749AP\xf1\x892943',
      b'\xf1\x8709S927158A \xf1\x893585',
      b'\xf1\x870CW300040H \xf1\x890606',
      b'\xf1\x870CW300041H \xf1\x891010',
      b'\xf1\x870CW300042F \xf1\x891604',
      b'\xf1\x870CW300043B \xf1\x891601',
      b'\xf1\x870CW300043E \xf1\x891603',
      b'\xf1\x870CW300044S \xf1\x894530',
      b'\xf1\x870CW300044T \xf1\x895245',
      b'\xf1\x870CW300045  \xf1\x894531',
      b'\xf1\x870CW300047D \xf1\x895261',
      b'\xf1\x870CW300048J \xf1\x890611',
      b'\xf1\x870D9300012  \xf1\x894904',
      b'\xf1\x870D9300012  \xf1\x894913',
      b'\xf1\x870D9300012  \xf1\x894937',
      b'\xf1\x870D9300012  \xf1\x895045',
      b'\xf1\x870D9300014M \xf1\x895004',
      b'\xf1\x870D9300014Q \xf1\x895006',
      b'\xf1\x870D9300020J \xf1\x894902',
      b'\xf1\x870D9300020Q \xf1\x895201',
      b'\xf1\x870D9300020S \xf1\x895201',
      b'\xf1\x870D9300040A \xf1\x893613',
      b'\xf1\x870D9300040S \xf1\x894311',
      b'\xf1\x870D9300041H \xf1\x895220',
      b'\xf1\x870D9300041P \xf1\x894507',
      b'\xf1\x870DD300045K \xf1\x891120',
      b'\xf1\x870DD300046F \xf1\x891601',
      b'\xf1\x870GC300012A \xf1\x891403',
      b'\xf1\x870GC300014B \xf1\x892401',
      b'\xf1\x870GC300014B \xf1\x892405',
      b'\xf1\x870GC300020G \xf1\x892401',
      b'\xf1\x870GC300020G \xf1\x892403',
      b'\xf1\x870GC300020G \xf1\x892404',
      b'\xf1\x870GC300020N \xf1\x892804',
      b'\xf1\x870GC300043T \xf1\x899999',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x875Q0959655AA\xf1\x890386\xf1\x82\x111413001113120043114317121C111C9113',
      b'\xf1\x875Q0959655AA\xf1\x890386\xf1\x82\x111413001113120053114317121C111C9113',
      b'\xf1\x875Q0959655AA\xf1\x890388\xf1\x82\x111413001113120043114317121C111C9113',
      b'\xf1\x875Q0959655AA\xf1\x890388\xf1\x82\x111413001113120043114417121411149113',
      b'\xf1\x875Q0959655AA\xf1\x890388\xf1\x82\x111413001113120053114317121C111C9113',
      b'\xf1\x875Q0959655BH\xf1\x890336\xf1\x82\x1314160011123300314211012230229333463100',
      b'\xf1\x875Q0959655BS\xf1\x890403\xf1\x82\x1314160011123300314240012250229333463100',
      b'\xf1\x875Q0959655BT\xf1\x890403\xf1\x82\x13141600111233003142404A2251229333463100',
      b'\xf1\x875Q0959655BT\xf1\x890403\xf1\x82\x13141600111233003142404A2252229333463100',
      b'\xf1\x875Q0959655BT\xf1\x890403\xf1\x82\x13141600111233003142405A2252229333463100',
      b'\xf1\x875Q0959655C \xf1\x890361\xf1\x82\x111413001112120004110415121610169112',
      b'\xf1\x875Q0959655D \xf1\x890388\xf1\x82\x111413001113120006110417121A101A9113',
      b'\xf1\x875Q0959655J \xf1\x890830\xf1\x82\x13271112111312--071104171825102591131211',
      b'\xf1\x875Q0959655J \xf1\x890830\xf1\x82\x13271212111312--071104171838103891131211',
      b'\xf1\x875Q0959655J \xf1\x890830\xf1\x82\x13341512112212--071104172328102891131211',
      b'\xf1\x875Q0959655J \xf1\x890830\xf1\x82\x13272512111312--07110417182C102C91131211',
      b'\xf1\x875Q0959655M \xf1\x890361\xf1\x82\x111413001112120041114115121611169112',
      b'\xf1\x875Q0959655S \xf1\x890870\xf1\x82\x1315120011211200621143171717111791132111',
      b'\xf1\x875Q0959655S \xf1\x890870\xf1\x82\x1324230011211200061104171724102491132111',
      b'\xf1\x875Q0959655S \xf1\x890870\xf1\x82\x1324230011211200621143171724112491132111',
      b'\xf1\x875Q0959655S \xf1\x890870\xf1\x82\x1315120011211200061104171717101791132111',
      b'\xf1\x875Q0959655S \xf1\x890870\xf1\x82\x1324230011211200631143171724122491132111',
      b'\xf1\x875Q0959655T \xf1\x890825\xf1\x82\x13271200111312--071104171837103791132111',
      b'\xf1\x875Q0959655T \xf1\x890830\xf1\x82\x13271100111312--071104171826102691131211',
      b'\xf1\x875QD959655  \xf1\x890388\xf1\x82\x111413001113120006110417121D101D9112',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x873Q0909144F \xf1\x895043\xf1\x82\x0561A01612A0',
      b'\xf1\x873Q0909144H \xf1\x895061\xf1\x82\x0566A0J612A1',
      b'\xf1\x873Q0909144J \xf1\x895063\xf1\x82\x0566A00514A1',
      b'\xf1\x873Q0909144J \xf1\x895063\xf1\x82\x0566A01613A1',
      b'\xf1\x873Q0909144J \xf1\x895063\xf1\x82\x0566A0J712A1',
      b'\xf1\x873Q0909144K \xf1\x895072\xf1\x82\x0571A0J714A1',
      b'\xf1\x873Q0909144L \xf1\x895081\xf1\x82\x0571A0JA15A1',
      b'\xf1\x873Q0909144M \xf1\x895082\xf1\x82\x0571A01A18A1',
      b'\xf1\x873Q0909144M \xf1\x895082\xf1\x82\x0571A0JA16A1',
      b'\xf1\x873QM909144  \xf1\x895072\xf1\x82\x0571A01714A1',
      b'\xf1\x875Q0909143K \xf1\x892033\xf1\x820519A9040203',
      b'\xf1\x875Q0909144AA\xf1\x891081\xf1\x82\x0521A00441A1',
      b'\xf1\x875Q0909144AA\xf1\x891081\xf1\x82\x0521A00608A1',
      b'\xf1\x875Q0909144AA\xf1\x891081\xf1\x82\x0521A00641A1',
      b'\xf1\x875Q0909144AB\xf1\x891082\xf1\x82\x0521A00442A1',
      b'\xf1\x875Q0909144AB\xf1\x891082\xf1\x82\x0521A00642A1',
      b'\xf1\x875Q0909144AB\xf1\x891082\xf1\x82\x0521A07B05A1',
      b'\xf1\x875Q0909144L \xf1\x891021\xf1\x82\x0521A00602A0',
      b'\xf1\x875Q0909144L \xf1\x891021\xf1\x82\x0522A00402A0',
      b'\xf1\x875Q0909144L \xf1\x891021\xf1\x82\x0521A00502A0',
      b'\xf1\x875Q0909144P \xf1\x891043\xf1\x82\x0511A00403A0',
      b'\xf1\x875Q0909144R \xf1\x891061\xf1\x82\x0516A00604A1',
      b'\xf1\x875Q0909144S \xf1\x891063\xf1\x82\x0516A00404A1',
      b'\xf1\x875Q0909144S \xf1\x891063\xf1\x82\x0516A00504A1',
      b'\xf1\x875Q0909144S \xf1\x891063\xf1\x82\x0516A00604A1',
      b'\xf1\x875Q0909144S \xf1\x891063\xf1\x82\x0516A07A02A1',
      b'\xf1\x875Q0909144T \xf1\x891072\xf1\x82\x0521A00507A1',
      b'\xf1\x875Q0909144T \xf1\x891072\xf1\x82\x0521A07B04A1',
      b'\xf1\x875Q0909144T \xf1\x891072\xf1\x82\x0521A20B03A1',
      b'\xf1\x875QD909144B \xf1\x891072\xf1\x82\x0521A00507A1',
      b'\xf1\x875QM909144A \xf1\x891072\xf1\x82\x0521A20B03A1',
      b'\xf1\x875QM909144B \xf1\x891081\xf1\x82\x0521A00442A1',
      b'\xf1\x875QN909144A \xf1\x895081\xf1\x82\x0571A01A16A1',
      b'\xf1\x875QN909144A \xf1\x895081\xf1\x82\x0571A01A18A1',
      b'\xf1\x875QN909144A \xf1\x895081\xf1\x82\x0571A01A17A1',
      b'\xf1\x875QN909144B \xf1\x895082\xf1\x82\x0571A01A18A1',
      b'\xf1\x875Q0910143C \xf1\x892211\xf1\x82\x0567A2000400',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x875Q0907567G \xf1\x890390\xf1\x82\x0101',
      b'\xf1\x875Q0907567J \xf1\x890396\xf1\x82\x0101',
      b'\xf1\x875Q0907572A \xf1\x890141\xf1\x82\x0101',
      b'\xf1\x875Q0907572B \xf1\x890200\xf1\x82\x0101',
      b'\xf1\x875Q0907572C \xf1\x890210\xf1\x82\x0101',
      b'\xf1\x875Q0907572D \xf1\x890304\xf1\x82\x0101',
      b'\xf1\x875Q0907572E \xf1\x89X310\xf1\x82\x0101',
      b'\xf1\x875Q0907572F \xf1\x890400\xf1\x82\x0101',
      b'\xf1\x875Q0907572G \xf1\x890571',
      b'\xf1\x875Q0907572H \xf1\x890620',
      b'\xf1\x875Q0907572J \xf1\x890654',
      b'\xf1\x875Q0907572P \xf1\x890682',
      b'\xf1\x875Q0907572R \xf1\x890771',
      b'\xf1\x875Q0907572S \xf1\x890780',
    ],
  },
  CAR.JETTA_MK7: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8704E906024AK\xf1\x899937',
      b'\xf1\x8704E906024AS\xf1\x899912',
      b'\xf1\x8704E906024BC\xf1\x899971',
      b'\xf1\x8704E906024BG\xf1\x891057',
      b'\xf1\x8704E906024B \xf1\x895594',
      b'\xf1\x8704E906024C \xf1\x899970',
      b'\xf1\x8704E906024L \xf1\x895595',
      b'\xf1\x8704E906024L \xf1\x899970',
      b'\xf1\x8704E906027MS\xf1\x896223',
      b'\xf1\x875G0906259T \xf1\x890003',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x8709G927158BQ\xf1\x893545',
      b'\xf1\x8709S927158BS\xf1\x893642',
      b'\xf1\x8709S927158BS\xf1\x893694',
      b'\xf1\x8709S927158CK\xf1\x893770',
      b'\xf1\x8709S927158R \xf1\x893552',
      b'\xf1\x8709S927158R \xf1\x893587',
      b'\xf1\x870GC300020N \xf1\x892803',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x875Q0959655AG\xf1\x890336\xf1\x82\02314171231313500314611011630169333463100',
      b'\xf1\x875Q0959655AG\xf1\x890338\xf1\x82\x1314171231313500314611011630169333463100',
      b'\xf1\x875Q0959655BM\xf1\x890403\xf1\x82\02314171231313500314642011650169333463100',
      b'\xf1\x875Q0959655BM\xf1\x890403\xf1\x82\02314171231313500314643011650169333463100',
      b'\xf1\x875Q0959655BR\xf1\x890403\xf1\x82\02311170031313300314240011150119333433100',
      b'\xf1\x875Q0959655BR\xf1\x890403\xf1\x82\02319170031313300314240011550159333463100',
      b'\xf1\x875Q0959655CB\xf1\x890421\xf1\x82\x1314171231313500314643021650169333613100',
      b'\xf1\x875Q0959655CB\xf1\x890421\xf1\x82\x1314171231313500314642021650169333613100',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x873Q0909144M \xf1\x895082\xf1\x82\x0571A10A11A1',
      b'\xf1\x875QM909144B \xf1\x891081\xf1\x82\00521A10A01A1',
      b'\xf1\x875QM909144B \xf1\x891081\xf1\x82\x0521B00404A1',
      b'\xf1\x875QM909144C \xf1\x891082\xf1\x82\00521A00642A1',
      b'\xf1\x875QM909144C \xf1\x891082\xf1\x82\00521A10A01A1',
      b'\xf1\x875QN909144B \xf1\x895082\xf1\x82\00571A10A11A1',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x875Q0907572N \xf1\x890681',
      b'\xf1\x875Q0907572P \xf1\x890682',
      b'\xf1\x875Q0907572R \xf1\x890771',
    ],
  },
  CAR.PASSAT_MK8: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8703N906026E \xf1\x892114',
      b'\xf1\x8704E906023AH\xf1\x893379',
      b'\xf1\x8704L906026DP\xf1\x891538',
      b'\xf1\x8704L906026ET\xf1\x891990',
      b'\xf1\x8704L906026FP\xf1\x892012',
      b'\xf1\x8704L906026GA\xf1\x892013',
      b'\xf1\x8704L906026KD\xf1\x894798',
      b'\xf1\x873G0906264  \xf1\x890004',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x870CW300043H \xf1\x891601',
      b'\xf1\x870CW300048R \xf1\x890610',
      b'\xf1\x870D9300013A \xf1\x894905',
      b'\xf1\x870D9300014L \xf1\x895002',
      b'\xf1\x870D9300041A \xf1\x894801',
      b'\xf1\x870DD300045T \xf1\x891601',
      b'\xf1\x870DL300011H \xf1\x895201',
      b'\xf1\x870GC300042H \xf1\x891404',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x873Q0959655AE\xf1\x890195\xf1\x82\r56140056130012416612124111',
      b'\xf1\x873Q0959655AF\xf1\x890195\xf1\x82\r56140056130012026612120211',
      b'\xf1\x873Q0959655AN\xf1\x890306\xf1\x82\r58160058140013036914110311',
      b'\xf1\x873Q0959655BA\xf1\x890195\xf1\x82\r56140056130012516612125111',
      b'\xf1\x873Q0959655BB\xf1\x890195\xf1\x82\r56140056130012026612120211',
      b'\xf1\x873Q0959655BK\xf1\x890703\xf1\x82\0165915005914001344701311442900',
      b'\xf1\x873Q0959655CN\xf1\x890720\xf1\x82\x0e5915005914001305701311052900',
      b'\xf1\x875Q0959655S \xf1\x890870\xf1\x82\02315120011111200631145171716121691132111',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x873Q0909144J \xf1\x895063\xf1\x82\x0566B00611A1',
      b'\xf1\x873Q0909144J \xf1\x895063\xf1\x82\x0566B00711A1',
      b'\xf1\x875Q0909143K \xf1\x892033\xf1\x820514B0060703',
      b'\xf1\x875Q0909143M \xf1\x892041\xf1\x820522B0060803',
      b'\xf1\x875Q0909143M \xf1\x892041\xf1\x820522B0080803',
      b'\xf1\x875Q0909144AB\xf1\x891082\xf1\x82\00521B00606A1',
      b'\xf1\x875Q0909144S \xf1\x891063\xf1\x82\00516B00501A1',
      b'\xf1\x875Q0909144T \xf1\x891072\xf1\x82\00521B00703A1',
      b'\xf1\x875Q0910143C \xf1\x892211\xf1\x82\x0567B0020600',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x873Q0907572A \xf1\x890126',
      b'\xf1\x873Q0907572A \xf1\x890130',
      b'\xf1\x873Q0907572B \xf1\x890192',
      b'\xf1\x873Q0907572C \xf1\x890195',
      b'\xf1\x873Q0907572C \xf1\x890196',
      b'\xf1\x875Q0907572R \xf1\x890771',
    ],
  },
  CAR.PASSAT_NMS: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8706K906016C \xf1\x899609',
      b'\xf1\x8706K906016G \xf1\x891124',
      b'\xf1\x8706K906071BJ\xf1\x894891',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x8709G927158AB\xf1\x893318',
      b'\xf1\x8709G927158BD\xf1\x893121',
      b'\xf1\x8709G927158FQ\xf1\x893745',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x87561959655  \xf1\x890210\xf1\x82\02212121111113000102011--121012--101312',
      b'\xf1\x87561959655C \xf1\x890508\xf1\x82\02215141111121100314919--153015--304831',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x87561907567A \xf1\x890132',
      b'\xf1\x877N0907572C \xf1\x890211\xf1\x82\00152',
    ],
  },
  CAR.POLO_MK6: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8704C906025H \xf1\x895177',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x870CW300050D \xf1\x891908',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x872Q0959655AJ\xf1\x890250\xf1\x82\x1248130411110416--04040404784811152H14',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x872Q1909144M \xf1\x896041',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x872Q0907572R \xf1\x890372',
    ],
  },
  CAR.SHARAN_MK2: {
    # ملاحظة: شاحنة Sharan Mk2 بنظام EPS وناقل الحركة الأوتوماتيكي DQ250 يتطلبان دعم KWP2000 لتحديد البصمة
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8704L906016HE\xf1\x894635',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x877N0959655D \xf1\x890016\xf1\x82\x0801100705----10--',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x877N0907572C \xf1\x890211\xf1\x82\x0153',
    ],
  },
  CAR.TAOS_MK1: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8704E906027NJ\xf1\x891445',
      b'\xf1\x8705E906013E \xf1\x891624',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x8709S927158BL\xf1\x893791',
      b'\xf1\x8709S927158FF\xf1\x893876',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x875Q0959655CB\xf1\x890421\xf1\x82\x1311111111333500314646021450149333613100',
      b'\xf1\x875Q0959655CE\xf1\x890421\xf1\x82\x1311110011333300314240021350139333613100',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x875QM909144C \xf1\x891082\xf1\x82\x0521060405A1',
      b'\xf1\x875QM909144C \xf1\x891082\xf1\x82\x0521060605A1',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x872Q0907572T \xf1\x890383',
    ],
  },
  CAR.TCROSS_MK1: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8704C906025AK\xf1\x897053',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x870CW300050E \xf1\x891903',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x872Q0959655AJ\xf1\x890250\xf1\x82\02212130411110411--04041104141311152H14',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x872Q1909144M \xf1\x896041',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x872Q0907572T \xf1\x890383',
    ],
  },
  CAR.TIGUAN_MK2: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8703N906026D \xf1\x893680',
      b'\xf1\x8704E906027NB\xf1\x899504',
      b'\xf1\x8704L906026EJ\xf1\x893661',
      b'\xf1\x8704L906027G \xf1\x899893',
      b'\xf1\x875N0906259  \xf1\x890002',
      b'\xf1\x875NA906259H \xf1\x890002',
      b'\xf1\x875NA907115E \xf1\x890005',
      b'\xf1\x8783A907115B \xf1\x890005',
      b'\xf1\x8783A907115F \xf1\x890002',
      b'\xf1\x8783A907115G \xf1\x890001',
      b'\xf1\x8783A907115K \xf1\x890001',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x8709G927158DT\xf1\x893698',
      b'\xf1\x8709G927158FM\xf1\x893757',
      b'\xf1\x8709G927158GC\xf1\x893821',
      b'\xf1\x8709G927158GD\xf1\x893820',
      b'\xf1\x8709G927158GM\xf1\x893936',
      b'\xf1\x870D9300043  \xf1\x895202',
      b'\xf1\x870DL300011N \xf1\x892001',
      b'\xf1\x870DL300011N \xf1\x892012',
      b'\xf1\x870DL300012P \xf1\x892103',
      b'\xf1\x870DL300013A \xf1\x893005',
      b'\xf1\x870DL300013G \xf1\x892119',
      b'\xf1\x870DL300013G \xf1\x892120',
      b'\xf1\x870DL300014C \xf1\x893703',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x875Q0959655AR\xf1\x890317\xf1\x82\02331310031333334313132573732379333313100',
      b'\xf1\x875Q0959655BJ\xf1\x890336\xf1\x82\x1312110031333300314232583732379333423100',
      b'\xf1\x875Q0959655BM\xf1\x890403\xf1\x82\02316143231313500314641011750179333423100',
      b'\xf1\x875Q0959655BT\xf1\x890403\xf1\x82\02312110031333300314240583752379333423100',
      b'\xf1\x875Q0959655BT\xf1\x890403\xf1\x82\02331310031333336313140013950399333423100',
      b'\xf1\x875Q0959655BT\xf1\x890403\xf1\x82\x1331310031333334313140013750379333423100',
      b'\xf1\x875Q0959655BT\xf1\x890403\xf1\x82\x1331310031333334313140573752379333423100',
      b'\xf1\x875Q0959655CB\xf1\x890421\xf1\x82\x1316143231313500314647021750179333613100',
      b'\xf1\x875Q0959655CG\xf1\x890421\xf1\x82\x1331310031333300314240024050409333613100',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x875Q0909143M \xf1\x892041\xf1\x820529A6060603',
      b'\xf1\x875Q0909144AB\xf1\x891082\xf1\x82\x0521A60604A1',
      b'\xf1\x875Q0910143C \xf1\x892211\xf1\x82\x0567A6000600',
      b'\xf1\x875QF909144A \xf1\x895581\xf1\x82\x0571A60834A1',
      b'\xf1\x875QF909144B \xf1\x895582\xf1\x82\00571A60634A1',
      b'\xf1\x875QF909144B \xf1\x895582\xf1\x82\x0571A62A32A1',
      b'\xf1\x875QM909144B \xf1\x891081\xf1\x82\x0521A60604A1',
      b'\xf1\x875QM909144C \xf1\x891082\xf1\x82\x0521A60604A1',
      b'\xf1\x875QM909144C \xf1\x891082\xf1\x82\00521A60804A1',
      b'\xf1\x875QM907144D \xf1\x891063\xf1\x82\x002SA6092SOM',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x872Q0907572AA\xf1\x890396',
      b'\xf1\x872Q0907572J \xf1\x890156',
      b'\xf1\x872Q0907572M \xf1\x890233',
      b'\xf1\x872Q0907572Q \xf1\x890342',
      b'\xf1\x872Q0907572R \xf1\x890372',
      b'\xf1\x872Q0907572T \xf1\x890383',
    ],
  },
  CAR.TOURAN_MK2: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8704L906026HM\xf1\x893017',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x870CW300041E \xf1\x891005',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x875Q0959655AS\xf1\x890318\xf1\x82\023363500213533353141324C4732479333313100',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x875Q0909143P \xf1\x892051\xf1\x820531B0062105',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x873Q0907572C \xf1\x890195',
    ],
  },
  CAR.TRANSPORTER_T61: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8704L906057AP\xf1\x891186',
      b'\xf1\x8704L906057N \xf1\x890413',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x870BT300012G \xf1\x893102',
      b'\xf1\x870BT300012E \xf1\x893105',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x872Q0959655AE\xf1\x890506\xf1\x82\02316170411110411--04041704161611152S1411',
      b'\xf1\x872Q0959655AF\xf1\x890506\xf1\x82\x1316171111110411--04041711121211152S1413',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x877LA909144F \xf1\x897150\xf1\x82\005323A5519A2',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x872Q0907572R \xf1\x890372',
    ],
  },
  CAR.TROC_MK1: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8705E906018AT\xf1\x899640',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x870CW300051M \xf1\x891925',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x875Q0959655CG\xf1\x890421\xf1\x82\x13111100123333003142404M1152119333613100',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x875Q0909144AB\xf1\x891082\xf1\x82\x0521060405A1',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x872Q0907572T \xf1\x890383',
    ],
  },
  CAR.AUDI_A3_MK3: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8704E906023AN\xf1\x893695',
      b'\xf1\x8704E906023AR\xf1\x893440',
      b'\xf1\x8704E906023BL\xf1\x895190',
      b'\xf1\x8704E906027CJ\xf1\x897798',
      b'\xf1\x8704L997022N \xf1\x899459',
      b'\xf1\x875G0906259A \xf1\x890004',
      b'\xf1\x875G0906259L \xf1\x890002',
      b'\xf1\x875G0906259Q \xf1\x890002',
      b'\xf1\x878V0906259F \xf1\x890002',
      b'\xf1\x878V0906259H \xf1\x890002',
      b'\xf1\x878V0906259J \xf1\x890002',
      b'\xf1\x878V0906259K \xf1\x890001',
      b'\xf1\x878V0906264B \xf1\x890003',
      b'\xf1\x878V0907115B \xf1\x890007',
      b'\xf1\x878V0907404A \xf1\x890005',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x870CW300044T \xf1\x895245',
      b'\xf1\x870CW300048  \xf1\x895201',
      b'\xf1\x870D9300012  \xf1\x894912',
      b'\xf1\x870D9300012  \xf1\x894931',
      b'\xf1\x870D9300012K \xf1\x894513',
      b'\xf1\x870D9300013B \xf1\x894931',
      b'\xf1\x870D9300041N \xf1\x894512',
      b'\xf1\x870D9300043T \xf1\x899699',
      b'\xf1\x870DD300046  \xf1\x891604',
      b'\xf1\x870DD300046A \xf1\x891602',
      b'\xf1\x870DD300046F \xf1\x891602',
      b'\xf1\x870DD300046G \xf1\x891601',
      b'\xf1\x870DL300012E \xf1\x892012',
      b'\xf1\x870GC300011  \xf1\x890403',
      b'\xf1\x870GC300013M \xf1\x892402',
      b'\xf1\x870GC300042J \xf1\x891402',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x875Q0959655AB\xf1\x890388\xf1\x82\0211111001111111206110412111321139114',
      b'\xf1\x875Q0959655AM\xf1\x890315\xf1\x82\x1311111111111111311411011231129321212100',
      b'\xf1\x875Q0959655AM\xf1\x890318\xf1\x82\x1311111111111112311411011531159321212100',
      b'\xf1\x875Q0959655AR\xf1\x890315\xf1\x82\x1311110011131115311211012331239321212100',
      b'\xf1\x875Q0959655BJ\xf1\x890339\xf1\x82\x1311110011131100311111011731179321342100',
      b'\xf1\x875Q0959655J \xf1\x890825\xf1\x82\x13111112111111--241115141112221291163221',
      b'\xf1\x875Q0959655J \xf1\x890825\xf1\x82\023111112111111--171115141112221291163221',
      b'\xf1\x875Q0959655J \xf1\x890830\xf1\x82\023121111111211--261117141112231291163221',
      b'\xf1\x875Q0959655J \xf1\x890830\xf1\x82\x13111112111111--241115141112221291163221',
      b'\xf1\x875Q0959655J \xf1\x890830\xf1\x82\x13121111111111--341117141212231291163221',
      b'\xf1\x875Q0959655N \xf1\x890361\xf1\x82\0211212001112110004110411111421149114',
      b'\xf1\x875Q0959655N \xf1\x890361\xf1\x82\0211212001112111104110411111521159114',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x873Q0909144H \xf1\x895061\xf1\x82\00566G0HA14A1',
      b'\xf1\x873Q0909144J \xf1\x895063\xf1\x82\x0566G0HA14A1',
      b'\xf1\x873Q0909144K \xf1\x895072\xf1\x82\x0571G01A16A1',
      b'\xf1\x873Q0909144K \xf1\x895072\xf1\x82\x0571G0HA16A1',
      b'\xf1\x873Q0909144L \xf1\x895081\xf1\x82\x0571G0JA14A1',
      b'\xf1\x875Q0909144AB\xf1\x891082\xf1\x82\00521G0G809A1',
      b'\xf1\x875Q0909144P \xf1\x891043\xf1\x82\00503G00303A0',
      b'\xf1\x875Q0909144P \xf1\x891043\xf1\x82\00503G00803A0',
      b'\xf1\x875Q0909144P \xf1\x891043\xf1\x82\x0503G0G803A0',
      b'\xf1\x875Q0909144R \xf1\x891061\xf1\x82\00516G00804A1',
      b'\xf1\x875Q0909144T \xf1\x891072\xf1\x82\00521G00807A1',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x875Q0907567N \xf1\x890400\xf1\x82\00101',
      b'\xf1\x875Q0907572D \xf1\x890304\xf1\x82\00101',
      b'\xf1\x875Q0907572G \xf1\x890571',
      b'\xf1\x875Q0907572H \xf1\x890620',
      b'\xf1\x875Q0907572P \xf1\x890682',
    ],
  },
  CAR.AUDI_Q2_MK1: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8704E906027JT\xf1\x894145',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x870CW300041F \xf1\x891006',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x875Q0959655BD\xf1\x890336\xf1\x82\x1311111111111100311211011231129321312111',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x873Q0909144K \xf1\x895072\xf1\x82\x0571F60511A1',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x872Q0907572M \xf1\x890233',
    ],
  },
  CAR.AUDI_Q3_MK2: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8705E906018N \xf1\x899970',
      b'\xf1\x8705L906022M \xf1\x890901',
      b'\xf1\x8783A906259  \xf1\x890001',
      b'\xf1\x8783A906259  \xf1\x890005',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x8709G927158CN\xf1\x893608',
      b'\xf1\x870GC300045D \xf1\x892802',
      b'\xf1\x870GC300046F \xf1\x892701',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x875Q0959655BF\xf1\x890403\xf1\x82\x1321211111211200311121232152219321422111',
      b'\xf1\x875Q0959655CC\xf1\x890421\xf1\x82\x131111111111120031111224118A119321532111',
      b'\xf1\x875Q0959655CC\xf1\x890421\xf1\x82\x131111111111120031111237116A119321532111',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x875Q0910143C \xf1\x892211\xf1\x82\x0567G6000300',
      b'\xf1\x875Q0910143C \xf1\x892211\xf1\x82\x0567G6000800',
      b'\xf1\x875QF909144B \xf1\x895582\xf1\x82\x0571G60533A1',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x872Q0907572R \xf1\x890372',
      b'\xf1\x872Q0907572T \xf1\x890383',
    ],
  },
  CAR.SEAT_ATECA_MK1: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8704E906027KA\xf1\x893749',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x870D9300014S \xf1\x895202',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x873Q0959655BH\xf1\x890703\xf1\x82\0161212001211001305121211052900',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x873Q0909144L \xf1\x895081\xf1\x82\00571N60511A1',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x872Q0907572M \xf1\x890233',
    ],
  },
  CAR.SEAT_LEON_MK3: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8704L906021EL\xf1\x897542',
      b'\xf1\x8704L906026BP\xf1\x891198',
      b'\xf1\x8704L906026BP\xf1\x897608',
      b'\xf1\x8704L906056CR\xf1\x892797',
      b'\xf1\x8705E906018AS\xf1\x899596',
      b'\xf1\x878V0906264H \xf1\x890005',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x870CW300041D \xf1\x891004',
      b'\xf1\x870CW300041G \xf1\x891003',
      b'\xf1\x870CW300050J \xf1\x891908',
      b'\xf1\x870D9300042M \xf1\x895016',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x873Q0959655AC\xf1\x890189\xf1\x82\r11110011110011021511110200',
      b'\xf1\x873Q0959655AS\xf1\x890200\xf1\x82\r11110011110011021511110200',
      b'\xf1\x873Q0959655AS\xf1\x890200\xf1\x82\r12110012120012021612110200',
      b'\xf1\x873Q0959655BH\xf1\x890703\xf1\x82\x0e1312001313001305171311052900',
      b'\xf1\x873Q0959655CM\xf1\x890720\xf1\x82\0161312001313001305171311052900',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x875Q0909144AB\xf1\x891082\xf1\x82\00521N01342A1',
      b'\xf1\x875Q0909144P \xf1\x891043\xf1\x82\00511N01805A0',
      b'\xf1\x875Q0909144T \xf1\x891072\xf1\x82\x0521N01309A1',
      b'\xf1\x875Q0909144T \xf1\x891072\xf1\x82\00521N05808A1',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x875Q0907572B \xf1\x890200\xf1\x82\00101',
      b'\xf1\x875Q0907572H \xf1\x890620',
      b'\xf1\x875Q0907572K \xf1\x890402\xf1\x82\x0101',
      b'\xf1\x875Q0907572P \xf1\x890682',
    ],
  },
  CAR.SKODA_FABIA_MK4: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8705E906018CF\xf1\x891905',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x870CW300051M \xf1\x891936',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x875QF959655AT\xf1\x890755\xf1\x82\x1311110011110011111100110200--1111120749',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x872Q1909144S \xf1\x896042',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x872Q0907572AA\xf1\x890396',
    ],
  },
  CAR.SKODA_KAMIQ_MK1: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8705C906032M \xf1\x891333',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x870CW300020  \xf1\x891906',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x872Q0959655AM\xf1\x890351\xf1\x82\0222221042111042121040404042E2711152H14',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x872Q1909144M \xf1\x896041',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x872Q0907572T \xf1\x890383',
    ],
  },
  CAR.SKODA_KAROQ_MK1: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8705E906018P \xf1\x896020',
      b'\xf1\x8705L906022BS\xf1\x890913',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x870CW300041S \xf1\x891615',
      b'\xf1\x870GC300014L \xf1\x892802',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x873Q0959655BH\xf1\x890712\xf1\x82\0161213001211001101131122012100',
      b'\xf1\x873Q0959655DE\xf1\x890731\xf1\x82\x0e1213001211001101131121012J00',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x875Q0910143C \xf1\x892211\xf1\x82\00567T6100500',
      b'\xf1\x875Q0910143C \xf1\x892211\xf1\x82\x0567T6100700',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x872Q0907572M \xf1\x890233',
      b'\xf1\x872Q0907572T \xf1\x890383',
    ],
  },
  CAR.SKODA_KODIAQ_MK1: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8704E906027DD\xf1\x893123',
      b'\xf1\x8704L906026DE\xf1\x895418',
      b'\xf1\x8704L906026EJ\xf1\x893661',
      b'\xf1\x8704L906026HT\xf1\x893617',
      b'\xf1\x8783A907115E \xf1\x890001',
      b'\xf1\x8705E906018DJ\xf1\x890915',
      b'\xf1\x875NA907115E \xf1\x890003',
      b'\xf1\x875NA907115E \xf1\x890005',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x870D9300043  \xf1\x895202',
      b'\xf1\x870DL300011N \xf1\x892014',
      b'\xf1\x870DL300012M \xf1\x892107',
      b'\xf1\x870DL300012N \xf1\x892110',
      b'\xf1\x870DL300013G \xf1\x892119',
      b'\xf1\x870GC300014N \xf1\x892801',
      b'\xf1\x870GC300046Q \xf1\x892802',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x873Q0959655AP\xf1\x890306\xf1\x82\r11110011110011421111314211',
      b'\xf1\x873Q0959655BJ\xf1\x890703\xf1\x82\x0e1213001211001205212111052100',
      b'\xf1\x873Q0959655BK\xf1\x890703\xf1\x82\x0e1213001211001244212111442100',
      b'\xf1\x873Q0959655CN\xf1\x890720\xf1\x82\x0e1213001211001205212112052100',
      b'\xf1\x873Q0959655CQ\xf1\x890720\xf1\x82\x0e1213111211001205212112052111',
      b'\xf1\x873Q0959655DJ\xf1\x890731\xf1\x82\x0e1513001511001205232113052J00',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x875Q0909143P \xf1\x892051\xf1\x820527T6050405',
      b'\xf1\x875Q0909143P \xf1\x892051\xf1\x820527T6060405',
      b'\xf1\x875Q0909143P \xf1\x892051\xf1\x820527T6070405',
      b'\xf1\x875Q0910143C \xf1\x892211\xf1\x82\x0567T600G500',
      b'\xf1\x875Q0910143C \xf1\x892211\xf1\x82\x0567T600G600',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x872Q0907572Q \xf1\x890342',
      b'\xf1\x872Q0907572R \xf1\x890372',
      b'\xf1\x872Q0907572T \xf1\x890383',
      b'\xf1\x872Q0907572AA\xf1\x890396',
      b'\xf1\x872Q0907572AB\xf1\x890397',
    ],
  },
  CAR.SKODA_OCTAVIA_MK3: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8704C906025L \xf1\x896198',
      b'\xf1\x8704E906016ER\xf1\x895823',
      b'\xf1\x8704E906027HD\xf1\x893742',
      b'\xf1\x8704E906027MH\xf1\x894786',
      b'\xf1\x8704L906021DT\xf1\x898127',
      b'\xf1\x8704L906026BS\xf1\x891541',
      b'\xf1\x875G0906259C \xf1\x890002',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x870CW300041L \xf1\x891601',
      b'\xf1\x870CW300041N \xf1\x891605',
      b'\xf1\x870CW300043B \xf1\x891601',
      b'\xf1\x870CW300043P \xf1\x891605',
      b'\xf1\x870D9300041C \xf1\x894936',
      b'\xf1\x870D9300041J \xf1\x894902',
      b'\xf1\x870D9300041P \xf1\x894507',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x873Q0959655AC\xf1\x890200\xf1\x82\r11120011100010022212110200',
      b'\xf1\x873Q0959655AQ\xf1\x890200\xf1\x82\r11120011100010312212113100',
      b'\xf1\x873Q0959655AS\xf1\x890200\xf1\x82\r11120011100010022212110200',
      b'\xf1\x873Q0959655BH\xf1\x890703\xf1\x82\0163221003221002105755331052100',
      b'\xf1\x873Q0959655CM\xf1\x890720\xf1\x82\x0e3221003221002105755331052100',
      b'\xf1\x873Q0959655CN\xf1\x890720\xf1\x82\x0e3221003221002105755331052100',
      b'\xf1\x875QD959655  \xf1\x890388\xf1\x82\x111101000011110006110411111111119111',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x873Q0909144J \xf1\x895063\xf1\x82\00566A01513A1',
      b'\xf1\x875Q0909144AA\xf1\x891081\xf1\x82\00521T00403A1',
      b'\xf1\x875Q0909144AB\xf1\x891082\xf1\x82\x0521T00403A1',
      b'\xf1\x875QD909144E \xf1\x891081\xf1\x82\x0521T00503A1',
      b'\xf1\x875Q0909144R \xf1\x891061\xf1\x82\x0516A00604A1',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x875Q0907572D \xf1\x890304\xf1\x82\x0101',
      b'\xf1\x875Q0907572F \xf1\x890400\xf1\x82\00101',
      b'\xf1\x875Q0907572J \xf1\x890654',
      b'\xf1\x875Q0907572P \xf1\x890682',
      b'\xf1\x875Q0907572R \xf1\x890771',
    ],
  },
  CAR.SKODA_SCALA_MK1: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8704C906025AK\xf1\x897053',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x870CW300050  \xf1\x891709',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x872Q0959655AJ\xf1\x890250\xf1\x82\x1211110411110411--04040404131111112H14',
      b'\xf1\x872Q0959655AM\xf1\x890351\xf1\x82\022111104111104112104040404111111112H14',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x872Q1909144M \xf1\x896041',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x872Q0907572R \xf1\x890372',
    ],
  },
  CAR.SKODA_SUPERB_MK3: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x8704L906026ET\xf1\x891343',
      b'\xf1\x8704L906026FP\xf1\x891196',
      b'\xf1\x8704L906026KB\xf1\x894071',
      b'\xf1\x8704L906026KD\xf1\x894798',
      b'\xf1\x873G0906259  \xf1\x890004',
      b'\xf1\x873G0906259B \xf1\x890002',
      b'\xf1\x873G0906264A \xf1\x890002',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x870CW300042H \xf1\x891601',
      b'\xf1\x870D9300011T \xf1\x894801',
      b'\xf1\x870D9300012  \xf1\x894940',
      b'\xf1\x870D9300041H \xf1\x894905',
      b'\xf1\x870GC300043  \xf1\x892301',
      b'\xf1\x870D9300043F \xf1\x895202',
    ],
    (Ecu.srs, 0x715, None): [
      b'\xf1\x875Q0959655AE\xf1\x890130\xf1\x82\x12111200111121001121110012211292221111',
      b'\xf1\x875Q0959655AE\xf1\x890130\xf1\x82\022111200111121001121118112231292221111',
      b'\xf1\x875Q0959655AK\xf1\x890130\xf1\x82\022111200111121001121110012211292221111',
      b'\xf1\x875Q0959655BH\xf1\x890336\xf1\x82\02331310031313100313131013141319331413100',
      b'\xf1\x875Q0959655CA\xf1\x890403\xf1\x82\x1331310031313100313151013141319331423100',
    ],
    (Ecu.eps, 0x712, None): [
      b'\xf1\x875Q0909143K \xf1\x892033\xf1\x820514UZ070203',
      b'\xf1\x875Q0909143M \xf1\x892041\xf1\x820522UZ070303',
      b'\xf1\x875Q0910143B \xf1\x892201\xf1\x82\00563UZ060700',
      b'\xf1\x875Q0910143B \xf1\x892201\xf1\x82\x0563UZ060600',
      b'\xf1\x875Q0910143C \xf1\x892211\xf1\x82\x0567UZ070600',
    ],
    (Ecu.fwdRadar, 0x757, None): [
      b'\xf1\x873Q0907572B \xf1\x890192',
      b'\xf1\x873Q0907572B \xf1\x890194',
      b'\xf1\x873Q0907572C \xf1\x890195',
    ],
  },
}
