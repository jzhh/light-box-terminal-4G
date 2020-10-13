#include "sunRiseSet.h"

static int days_of_month_1[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

static int days_of_month_2[] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

const static double h = -0.833;//日出日落时太阳的位置

const static double UTo = 180.0;//上次计算的日落日出时间，初始迭代值180.0

const uint8_t ret_ok = 0;
const uint8_t ret_err = 1;

static const double PI = 3.141592653589793;

//输入日期

//输入经纬度

//判断是否为闰年：若为闰年，返回1；若不是闰年,返回0

static uint8_t leap_year(int year) {
	if(year%4==0) 
	{ 
		if(year%100==0) 
		{ 
			if(year%400==0)return 1;
			else return 0;   
		}else return 1;   
	}else return 0;	

}

//求从格林威治时间公元2000年1月1日到计算日天数days

static int days(int year, int month, int date) {

		int i, a = 0;

		for (i = 2000; i < year; i++) {

				if (leap_year(i)) a = a + 366;

				else a = a + 365;

		}

		if (leap_year(year)) {

				for (i = 0; i < month - 1; i++) {

						a = a + days_of_month_2[i];

				}

		} else {

				for (i = 0; i < month - 1; i++) {

						a = a + days_of_month_1[i];

				}

		}

		a = a + date;

		return a;

}

//求格林威治时间公元2000年1月1日到计算日的世纪数t

static double t_century(int days, double UTo) {

		return ((double) days + UTo / 360) / 36525;

}

//求太阳的平黄径

static double L_sun(double t_century) {

		return (280.460 + 36000.770 * t_century);

}

//求太阳的平近点角

static double G_sun(double t_century) {

		return (357.528 + 35999.050 * t_century);

}

//求黄道经度

static double ecliptic_longitude(double L_sun, double G_sun) {

		return (L_sun + 1.915 * sin(G_sun * PI / 180) + 0.02 * sin(2 * G_sun * PI / 180));

}

//求地球倾角

static double earth_tilt(double t_century) {

		return (23.4393 - 0.0130 * t_century);

}

//求太阳偏差

static double sun_deviation(double earth_tilt, double ecliptic_longitude) {

		return (180 / PI * asin(sin(PI / 180 * earth_tilt) * sin(PI / 180 * ecliptic_longitude)));

}

//求格林威治时间的太阳时间角GHA

static double GHA(double UTo, double G_sun, double ecliptic_longitude) {

		return (UTo - 180 - 1.915 * sin(G_sun * PI / 180) - 0.02 * sin(2 * G_sun * PI / 180) + 
						2.466 * sin(2 * ecliptic_longitude * PI / 180) - 0.053 * sin(4 * ecliptic_longitude * PI / 180));

}

//求修正值e

static double e(double h, double glat, double sun_deviation) {

		return 180 / PI * acos((sin(h * PI / 180) - sin(glat * PI / 180) * sin(sun_deviation * PI / 180)) / (cos(glat * PI / 180) * cos(sun_deviation * PI / 180)));

}

//求日出时间

static double UT_rise(double UTo, double GHA, double glong, double e) {

		return (UTo - (GHA + glong + e));

}

//求日落时间

static double UT_set(double UTo, double GHA, double glong, double e) {

		return (UTo - (GHA + glong - e));

}

//判断并返回结果（日出）

static double result_rise(double UT, double UTo, double glong, double glat, int year, int month, int date) {

		double d;

		if (UT >= UTo) d = UT - UTo;

		else d = UTo - UT;

		if (d >= 0.1) {

				UTo = UT;

				UT = UT_rise(UTo,

								GHA(UTo, G_sun(t_century(days(year, month, date), UTo)),

												ecliptic_longitude(L_sun(t_century(days(year, month, date), UTo)),

																G_sun(t_century(days(year, month, date), UTo)))),

								glong,

								e(h, glat, sun_deviation(earth_tilt(t_century(days(year, month, date), UTo)),

												ecliptic_longitude(L_sun(t_century(days(year, month, date), UTo)),

																G_sun(t_century(days(year, month, date), UTo))))));

				result_rise(UT, UTo, glong, glat, year, month, date);


		}

		return UT;

}

//判断并返回结果（日落）

static double result_set(double UT, double UTo, double glong, double glat, int year, int month, int date) {

		double d;

		if (UT >= UTo) d = UT - UTo;

		else d = UTo - UT;

		if (d >= 0.1) {

				UTo = UT;

				UT = UT_set(UTo,

								GHA(UTo, G_sun(t_century(days(year, month, date), UTo)),

												ecliptic_longitude(L_sun(t_century(days(year, month, date), UTo)),

																G_sun(t_century(days(year, month, date), UTo)))),

								glong,

								e(h, glat, sun_deviation(earth_tilt(t_century(days(year, month, date), UTo)),

												ecliptic_longitude(L_sun(t_century(days(year, month, date), UTo)),

																G_sun(t_century(days(year, month, date), UTo))))));

				result_set(UT, UTo, glong, glat, year, month, date);

		}

		return UT;

}

//求时区

static int Zone(double glong) {

		if (glong >= 0) return (int) ((int) (glong / 15.0) + 1);

		else return (int) ((int) (glong / 15.0) - 1);

}

//打印结果

// public static void output(double rise, double set, double glong){

//     if((int)(60*(rise/15+Zone(glong)-(int)(rise/15+Zone(glong))))<10)

//         System.out.println("The time at which the sunrise is: "+(int)(rise/15+Zone(glong))+":"+(int)(60*(rise/15+Zone(glong)-(int)(rise/15+Zone(glong))))+" .\n");

//     else System.out.println("The time at which the sunrise is: "+(int)(rise/15+Zone(glong))+":"+(int)(60*(rise/15+Zone(glong)-(int)(rise/15+Zone(glong))))+" .\n");

//

//     if((int)(60*(set/15+Zone(glong)-(int)(set/15+Zone(glong))))<10)

//         System.out.println("The time at which the sunset is: "+(int)(set/15+Zone(glong))+": "+(int)(60*(set/15+Zone(glong)-(int)(set/15+Zone(glong))))+" .\n");

//     else System.out.println("The time at which the sunset is: "+(int)(set/15+Zone(glong))+":"+(int)(60*(set/15+Zone(glong)-(int)(set/15+Zone(glong))))+" .\n");

// }

uint8_t getSunrise(double glong, double glat, _calendar_obj calendar, RTC_TimeTypeDef *sunRiseTime) {
		if (calendar.w_year != 0 && calendar.w_month != 0 && calendar.w_date != 0 && 
																glong != 0 && glat != 0 && sunRiseTime != NULL) {
				double sunrise;
				int year, month, date;
				//SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd");
				//String dateTime = sdf.format(sunTime);
				//String[] rq = dateTime.split("-");
				//String y = rq[0];
				//String m = rq[1];
				//String d = rq[2];
				year = calendar.w_year;

				month = calendar.w_month;

				date = calendar.w_date;

				//glong = longitude.doubleValue();

				//glat = latitude.doubleValue();

				sunrise = result_rise(UT_rise(UTo,

								GHA(UTo, G_sun(t_century(days(year, month, date), UTo)),

												ecliptic_longitude(L_sun(t_century(days(year, month, date), UTo)),

																G_sun(t_century(days(year, month, date), UTo)))),

								glong,

								e(h, glat, sun_deviation(earth_tilt(t_century(days(year, month, date), UTo)),

												ecliptic_longitude(L_sun(t_century(days(year, month, date), UTo)),

																G_sun(t_century(days(year, month, date), UTo)))))), UTo, glong, glat, year, month, date);

//System.out.println("Sunrise is: "+(int)(sunrise/15+Zone(glong))+":"+(int)(60*(sunrise/15+Zone(glong)-(int)(sunrise/15+Zone(glong))))+" .\n");

//        Log.d("Sunrise", "Sunrise is: "+(int)(sunrise/15+8)+":"+(int)(60*(sunrise/15+8-(int)(sunrise/15+8)))+" .\n");

				//return "Sunrise is: "+(int)(sunrise/15+Zone(glong))+":"+(int)(60*(sunrise/15+Zone(glong)-(int)(sunrise/15+Zone(glong))))+" .\n";
				sunRiseTime->Hours = (sunrise / 15 + 8);
				sunRiseTime->Minutes = (60 * (sunrise / 15 + 8 - (int) (sunrise / 15 + 8)));
				sunRiseTime->Seconds = 0;
				return ret_ok;
		}
		return ret_err;
}


uint8_t getSunset(double glong, double glat, _calendar_obj calendar, RTC_TimeTypeDef *sunSetTime) {
		if (calendar.w_year != 0 && calendar.w_month != 0 && calendar.w_date != 0 && 
																glong != 0 && glat != 0 && sunSetTime != NULL) {
				double sunset;
				int year, month, date;
			
				year = calendar.w_year;

				month = calendar.w_month;

				date = calendar.w_date;

				sunset = result_set(UT_set(UTo,

								GHA(UTo, G_sun(t_century(days(year, month, date), UTo)),

												ecliptic_longitude(L_sun(t_century(days(year, month, date), UTo)),

																G_sun(t_century(days(year, month, date), UTo)))),

								glong,

								e(h, glat, sun_deviation(earth_tilt(t_century(days(year, month, date), UTo)),

												ecliptic_longitude(L_sun(t_century(days(year, month, date), UTo)),

																G_sun(t_century(days(year, month, date), UTo)))))), UTo, glong, glat, year, month, date);

//System.out.println("The time at which the sunset is: "+(int)(sunset/15+Zone(glong))+":"+(int)(60*(sunset/15+Zone(glong)-(int)(sunset/15+Zone(glong))))+" .\n");

//        Log.d("Sunset", "Sunset is: "+(int)(sunset/15+8)+":"+(int)(60*(sunset/15+8-(int)(sunset/15+8)))+" .\n");

				//return "Sunset is: "+(int)(sunset/15+Zone(glong))+":"+(int)(60*(sunset/15+Zone(glong)-(int)(sunset/15+Zone(glong))))+" .\n";

				//return (int) (sunset / 15 + 8) + ":" + (int) (60 * (sunset / 15 + 8 - (int) (sunset / 15 + 8)));
				sunSetTime->Hours = (sunset / 15 + 8);
				sunSetTime->Minutes = (60 * (sunset / 15 + 8 - (int) (sunset / 15 + 8)));
				sunSetTime->Seconds = 0;
				return ret_ok;
		}
		return ret_err;
}

/*
public static void main(String[] args) {
		String str1 = SunRiseSet.getSunrise(new BigDecimal(87.617161),new BigDecimal(43.82582),new Date());
		String str2 = SunRiseSet.getSunset(new BigDecimal(87.617161),new BigDecimal(43.82582),new Date());
		System.out.println("日出时间：" + str1);
		System.out.println("日落时间：" + str2);
}
*/