#include "GPS.h"

void GPS::begin(){
	Serial.begin(9600);
  	Serial1.begin(9600);
  	delay(1000);
}

void GPS::getOrigInfo()
{
    double addLongitude = 0;
    double addLatitude = 0;
    for (int i = 0; i < 20; i ++) {
        getRawData();
        if (i >= 10) {
            addLongitude += curLongitude;
            addLatitude += curLatitude;
        }
        delay(20);
    }
    origLatitude = addLongitude / 10;
    origLongitude = addLatitude / 10;
}

double GPS::getGpsTime()
{
	return numUTCtime;
}

double GPS::getLatitude()
{
	return curLatitude;
}

double GPS::getLongitude()
{
	return curLongitude;
}

double GPS::getSpeed()
{
	return numSpeed;
}

double GPS::getOriginDistance()
{
	double currentRadLat = 0;
	double currentRadLong = 0;
	double oriRadLat = 0;
	double oriRadLong = 0;
	double difRadLat = 0;
	double difRadLong = 0;
  double s = 0;
	double distance = 0;
	currentRadLat = curLatitude*PI / 180;
	currentRadLong = curLongitude*PI / 180;
	oriRadLat = origLatitude*PI / 180;
	oriRadLong = origLongitude*PI / 180;
	difRadLat = currentRadLat - oriRadLat;
	difRadLong = currentRadLong - oriRadLong;
	s = (2 * asin(sqrt(pow(sin(difRadLat/2),2)+cos(currentRadLat)*cos(oriRadLat)*pow(sin(difRadLong/2),2))))*EARTH_RADIUS;
	distance = round(s*10000.0) / 10000.0 * 1000.0;
	return distance;
}	

double GPS::getAzimuth()
{
	  double currentRadLat = 0;
    double currentRadLong = 0;
    double oriRadLat = 0;
    double oriRadLong = 0;
    double Azimuth;
    currentRadLat = curLatitude*PI / 180;
    currentRadLong = curLongitude*PI / 180;
    oriRadLat = origLatitude*PI / 180;
    oriRadLong = origLongitude*PI / 180;
    Azimuth = sin(oriRadLat)*sin(currentRadLat)+cos(oriRadLat)*cos(currentRadLat)*cos(currentRadLong-oriRadLong);
    Azimuth = sqrt(1-Azimuth*Azimuth);
    Azimuth = cos(currentRadLat)*sin(currentRadLong-oriRadLong)/Azimuth;
    Azimuth = asin(Azimuth)*180/PI;
    if ((curLatitude > origLatitude)&&(curLongitude > origLongitude)){
    }
    else if((curLatitude == origLatitude) && (curLongitude > origLongitude))
    {
        return 90;
    }
    else if(curLatitude < origLatitude){
        return 180-Azimuth;
    }
    else if ((curLatitude == origLatitude) && (curLongitude < origLongitude)){
        return 270;
    }
    else if((curLatitude > origLatitude)&&(curLongitude < origLongitude)){
        return 360+Azimuth;
    }
    return Azimuth;
}



void GPS::getRawData()
{
	while (Serial1.available()> 0)
	{
	    if(Mark_Start)
	    {
	      data=reader();//输出数据的类型 是从哪个服务器获取的数据 reader 第一个总会先读出类型
	        if(data.equals("GNRMC"))
	        {
	        RMCUTCtime=reader();//先读时间
	        numUTCtime=RMCUTCtime.toDouble();//转换成Double形式
	        PositionValid=reader();//读是A还是E
	        RMClatitude=reader();//纬度数字(任然是字符串形式)
	        numLatitude=RMClatitude.toDouble();//转换成Double形式
          curLatitude=floor(numLatitude/100.0)+fmod(numLatitude,100.0)/60.0;
	        RMClatitude+=reader();//纬度编号
	        RMClongitude=reader();//精度数字
	        numLongitude=RMClongitude.toDouble();//转换成Double形式
          curLongitude=floor(numLongitude/100.0)+fmod(numLongitude,100.0)/60.0;
	        RMClongitude+=reader();//精度标号
	        Speed=reader();//速度
	        numSpeed=Speed.toDouble();//转换成Double形式
	        Direction=reader();//方向
	        Date=reader();//日期
	        Declination=reader();//磁偏角
	        Declination+=reader();//加方向
	        Mode=reader();//模式
	        valid=true;
	        Mark_Start=false;
	        data="";//让data归零
	        }
	      else{
	        Mark_Start=false;
	        data="";
	      }
	    }
	    if(valid)
	    {
	      if(PositionValid=="A")
	      {
	        Serial.println("Position Valid");
	      }
	      else
	      {
	        Serial.println("Your position is not valid.");
	      }
	      Serial.print("Date:");
	      Serial.println(Date);
	      Serial.print("UTCtime:");
	      Serial.print(RMCUTCtime);
	      Serial.println("   ");
	      Serial.print("Latitude:");
	      Serial.print(RMClatitude);
	      Serial.println("   ");
	      Serial.print("Longitude:");
	      Serial.print(RMClongitude);
	      Serial.println("   ");
	      Serial.print("Speed:");
	      Serial.println(Speed);
	      Serial.print("Direction:");
	      Serial.println(Direction);
	      Serial.print("Declination:");
	      Serial.println(Declination);
	      Serial.print("Mode:");
	      Serial.println(Mode);     
	      valid=false;
	    }
	     if(Serial1.find('$'))
	     { //Serial.find() reads data from the serial buffer until the target string of given length is found
	      Serial.println("capture");//捕捉到开头
	      Mark_Start=true;
	     }
	}
}

String GPS::reader()
{
	 String value="";
	  int temp; //temp 类型默认为int  很重要 因为用read读取出来的数值不一定只有一位，read是读一个byte，逗号这种算一个byte 但是一个byte不仅仅有一个数字
	startover: //返回位置
	  while (Serial1.available() > 0)
	  {
	    delay(2);
	    temp=Serial1.read();//用read取走一个值 the first byte of incoming serial data available (or -1 if no data is available) - int
	    if((temp==',')||(temp=='*')){//如果读到 “，”或“*”， 开始检查是否两个逗号或者乘号之间有value, 就是检查value有没有被存入东西
	      if(value.length()){//如果有 就返回value
	        //Serial.println("meaningful message");
	        return value;
	      }
	      else {
	        //Serial.println("empty");//没有就打EMPTY， 返回空值
	        return "";//跳出reader（）
	      }     
	    }
	    else if(temp=='$'){//如果读到钱号，说明读到下一个字符串了，或者说明连续读到两个钱号（个人理解：loop速度太快，reader还没读取下个BYTE）
	      //Serial.println("failure");
	      Mark_Start=false;//这里要把markstart设成false（每次reader一定会让markstart成为false 其实mainloop里面也补了一次赋值false） 然后继续loop 直到value
	    }
	    else{//如果又没有读到钱号又没有读到逗号 说明是内容 serial.reader 每次只读出一个字符 所以要用相加
	      //Serial.println("add"); 
	     value+=char(temp);//把数据变成char，再加到字符串里面
	    }
	  }
	  while (!(Serial1.available()>0)){
	  }
	  goto startover; //如果没读到数据 返回， 进行下一个char的读取
}
