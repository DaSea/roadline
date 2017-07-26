/*
 * Copyright (C) 2016-01-27 Dasea <dhf0214@126.com>
 *
 * Description: 全局参数的一个保存
 */

#ifndef __GLOBAL_SET__
#define __GLOBAL_SET__

class GlobalSet {
public:
    // setter
    static void SetCircleLineGap(double lineGap); // 圆曲线
    static void SetCurveLineGap(double lineGap); // 缓和曲线

    // getter
    static double GetCircleLineGap();  // 圆曲线
    static double GetCurveLineGap();   // 缓和曲线

private:
    static double _circleLineGap;
    static double _curveLineGap;
};

#endif
