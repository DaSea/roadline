/*
 * Copyright (C) 2016-01-26 Dasea <dhf0214@126.com>
 *
 * Description: 通用的算法类
 */
#include "MathFunction.h"
#include <cstdlib>
#include <cstring>

MathFunction::MathFunction()
{
}

MathFunction::~MathFunction()
{
}

double MathFunction::GetTwoPointLength(const Point3D& startPt, const Point3D& endPt)
{
    return sqrt((startPt.x-endPt.x)*(startPt.x-endPt.x)
            + (startPt.y-endPt.y)*(startPt.y-endPt.y));
}


// 计算线段的角度，通过第一，二想先弧度可以0到PI,如果在第三，四象限，弧度可以0到-PI
double MathFunction::GetLineAngle(const Point3D& startPt, const Point3D& endPt)
{
    double dx=endPt.x-startPt.x;
    double dy=endPt.y-startPt.y;
    double angle = atan(fabs(dy/dx)); // 0-PI/2
    if (0 < dx) { // 第一或第四象限
        if (0 > dy) { // 第四
            angle = 2.0*PI - angle;
        }
    } else if (0 > dx) { // 第二或第三象限
        if (0 <= dy) { // 第二
            angle = PI - angle;
        }  else {
            angle = PI + angle;
        }
    } else {
        if (0 < dy) {
            angle = PI/2.0;    
        } else if (0 > dy) {
            angle = PI * 1.5;
        }
    }
    return angle;
}

Point3D MathFunction::GetCircleCenter(const Point3D& startPt,
                                      const Point3D& middlePt,
                                      const Point3D& endPt){
    Point3D centerPt;

    // 三个点到圆心的距离等于半径:
    // (x1-x)^2 + (y1 - y)^2 = (x2 - x)^2 + (y2 - y)^2 = (x3 - x)^2 + (y3 - y)^2
    // 可以推出公式:
    // 2(x2 - x1)x + 2(y2 - y1)y = x2^2 + y2^2 - x1^2 - y1^2[ax + by = c]
    // 2(x3 - x2)x + 2(y3 - y2)y = x3^2 + y3^2 - x2^2 - y2^2[dx + ey = f]
    // 解二元一次方程:
    // x = (bf - ce) / (bd - ea)
    // y = (af - cd) / (ae - bd)
    double a = 2 * (middlePt.x - startPt.x);
    double b = 2 * (middlePt.y - startPt.y);
    double c = middlePt.x*middlePt.x + middlePt.y*middlePt.y - startPt.x*startPt.x - startPt.y*startPt.y;
    double d = 2 * (endPt.x - middlePt.x);
    double e = 2 * (endPt.y - middlePt.y);
    double f = endPt.x*endPt.x + endPt.y*endPt.y - middlePt.x*middlePt.x - middlePt.y*middlePt.y;

    double x = (b*f - c*e) / (b*d - e*a);
    double y = (a*f - c*d) / (a*e - b*d);

    centerPt.x = x;
    centerPt.y = y;

    return (centerPt);
}


//度分秒转换为度。
double MathFunction::ConvertDMSToD(const std::string& strDMS)
{
    double dms = atof(strDMS.c_str());
    return (DmsToDegree(dms));
}

double MathFunction::DegreeToDms(double V)
{
    // return CcurrencyFunc::DegreeToDms(V);
    //    看你的本意应该是48.713140 E,31.22908 N对吧。
    //    转化为度分秒，按照1度=60分，1分=60秒。
    //    48.713140 E就是48度，60×0.71314=42.7884分，取整故分为42分，60×0.7884=47.304，取整故秒为47秒。
    //    所以48.713140 E转换为度分秒即为东经48度42分47秒。
    int D = 0, M = 0;
    D = fabs(V);

    double ss = V - D;
    ss = ss * 60;
    M = fabs(ss);

    double mm = ss - M;
    mm = mm * 60;

    double result = (double)D + (double)M/100 + (double)mm/10000;

    return (result);
}

double MathFunction::DmsToDegree(double V)
{
    int D, M;
    double S, Rv;
    char Buf[20], *ptr;
    char tmp[10];

    Rv = fabs(V);
    sprintf(Buf, "%.10f", Rv);
    ptr = strchr(Buf, '.');
    *ptr = 0;

    D =(int) atof(Buf);
    tmp[0] = *(ptr+1); tmp[1] = *(ptr+2); tmp[2] = 0; ptr += 2;
    M =(int) atof(tmp);
    S = (Rv - D - M / 100.0) * 10000;

    Rv = D + M / 60.0 + S / 3600.0;
    if (V < 0) Rv = -Rv;

    return Rv;
}

//计算两条线段之间的夹角
double  MathFunction::GetTwoLineAngle(const Point3D& startPt,const Point3D& middlePt,const Point3D& endPt)
{
    //A 为夹角点
    double dx1 = startPt.x - middlePt.x;
    double dy1 = startPt.y - middlePt.y;

    double dx2 = endPt.x - middlePt.x;
    double dy2 = endPt.y - middlePt.y;

    double dx3=  startPt.x-endPt.x;
    double dy3=  startPt.y-endPt.y;

    double AB=sqrt(dx1 * dx1+ dy1 * dy1);
    double AC=sqrt(dx2 * dx2+ dy2 * dy2);
    double BC=sqrt(dx3 * dx3+ dy3 * dy3);

    double dRadio=acos((AB * AB + AC * AC-BC *BC)/(2*AB*AC));
    double dAngle=RADIAN((180-ANGLE(dRadio)));

    return dAngle;
}

//计算缓和曲线的内移值
double  MathFunction::GetCurveP(const double& curveLen, const  double& radius)
{
    return pow(curveLen,2)/(24*radius) - pow(curveLen,4)/(2384.0*pow(radius,3));
}

//计算缓和曲线的切线增量
double  MathFunction::GetCurveM(const double& curveLen, const  double& radius)
{
    return  curveLen/2-pow(curveLen,3)/(240*pow(radius,2));
}


/**
 * @brief  数的正负号
 *
 * @param  num 数
 *
 * @return 0, -1, 1
 */
int MathFunction::Sgn(double num) {
    if(0.0 == num){
        return 0;
    }else if(0.0 > num){
        return -1;
    }else{
        return 1;
    }
}


/**
 * @brief  计算线在拐点处道路的方向，左转还是右转在原方向左侧的时候为左转角，否则为右转角
 *
 * @param  aPt 顺着线路方向的第一个交点
 * @param  bPt 顺着线路方向的第二个交点
 * @param  cPt 顺着线路方向的第三个交点
 *
 * @return 偏向(-1: 左, 1: 右; 0:不偏)
 */
double MathFunction::ComputerDirection(Point3D& aPt, Point3D& bPt, Point3D& cPt)
{
    double direction = 0.0;
    double angleOne = GetLineAngle(aPt, bPt);
    double angleTwo = GetLineAngle(bPt, cPt);
    double dangle = angleTwo - angleOne;
    if (dangle > 0) {
        if (dangle > PI) {
            direction = -1; // 左转
        } else {
            direction = 1; // 右转
        }
    } else if (dangle < 0){
        dangle = 2*PI + dangle;
        if (dangle > PI) {
            direction = -1; // 左转;
        } else {
            direction = 1; // 右转
        }
    }

    return (direction);
}

double MathFunction::ComputerDirection(double angleOne, double angleTwo) {
    double direction = 0.0;
    double dangle = angleTwo - angleOne;
    if (dangle > 0) {
        if (dangle > PI) {
            direction = -1; // 左转
        } else {
            direction = 1; // 右转
        }
    } else if (dangle < 0){
        dangle = 2*PI + dangle;
        if (dangle > PI) {
            direction = -1; // 左转;
        } else {
            direction = 1; // 右转
        }
    }
    return (direction);
}

/**
 * @brief  判断输入的方位角值是否在0-2PI之间, 并校正
 *
 * @param  azimuth in, 待校正的值
 *
 * @return 校正后的值
 */
double MathFunction::JudgeAzimuth(double azimuth) {
    if(azimuth < 0) {
        return (azimuth + 2*PI);
    }

    if(azimuth > 2*PI) {
        return (azimuth - 2*PI);
    }

    return (azimuth);
}
