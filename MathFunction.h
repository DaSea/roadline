#ifndef __MATH_FUNCTION_H__
#define __MATH_FUNCTION_H__

#include "BaseGeometry.h"
#include <cmath>
#include <string>

// 共用函数类(interface)
class MathFunction {
public:
    MathFunction();
    ~MathFunction();

public:
    static double GetTwoPointLength(const Point3D& startPt, const Point3D& endPt);
    //计算线段的角度，通过第一，二想先弧度可以0到PI,如果在第三，四象限，弧度可以0到-PI
    static double GetLineAngle(const Point3D& startPt, const Point3D& endPt);

    // 根据三个点求圆心
    static Point3D GetCircleCenter(const Point3D& startPt, const Point3D& middlePt, const Point3D& endPt);

    //弧度转换为度
    static double ConvertRadianToDegree(const double& dRadian){return ANGLE(dRadian);};
    //角度转换为弧度
    static double ConVertDegreeToRadian(const double& dDegree){return RADIAN(dDegree);};
    //计算两条线之间的夹角
    static double GetTwoLineAngle(const Point3D& startPt,const Point3D& middlePt,const Point3D& endPt);

    // 计算缓和曲线的内移值
    static double  GetCurveP(const double& dCurveLengthOne, const  double& dCircleRadioLength);
    //计算缓和曲线的切线增量
    static double  GetCurveM(const double& dCurveLengthOne, const  double& dCircleRadioLength);
    // 计算偏向
    static double ComputerDirection(Point3D& aPt, Point3D& bPt, Point3D& cPt);
    static double ComputerDirection(double angleOne, double angleTwo);

    // 度分秒与du的互相转换
    static double ConvertDMSToD(const std::string& strDMS);
    static double DmsToDegree(double V); // 度.分秒->度
    static double DegreeToDms(double V); // 度->度.分秒 ddd.mmss

    // 提取一个数的正负号
    static int Sgn(double num);

    // 方位角的值的判断(0 - 360)
    static double JudgeAzimuth(double azimuth);
};

#endif /* __MATH_FUNCTION_H__ */
