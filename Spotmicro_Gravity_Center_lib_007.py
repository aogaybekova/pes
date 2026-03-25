#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from math import sqrt
import Spotmicro_lib_020
Spot = Spotmicro_lib_020.Spot()


class SpotCG:
    def CG_calculation (self,thetalf,thetarf,thetarr,thetalr):
        leg_cg_data = (
            (Spot.FK_Weight(thetalf, 1), (Spot.xlf, Spot.ylf, Spot.zlf)),
            (Spot.FK_Weight(thetarf, -1), (Spot.xrf, Spot.yrf, Spot.zrf)),
            (Spot.FK_Weight(thetarr, -1), (Spot.xrr, Spot.yrr, Spot.zrr)),
            (Spot.FK_Weight(thetalr, 1), (Spot.xlr, Spot.ylr, Spot.zlr)),
        )
        weights = (Spot.Weight_Shoulder, Spot.Weight_Leg, Spot.Weight_Foreleg)
        cg_indexes = ((0, 1, 2), (3, 4, 5), (6, 7, 8))
        body_cg = (Spot.xCG_Body, Spot.yCG_Body, Spot.zCG_Body)
        weight_sum = Spot.Weight_Body + 4 * sum(weights)

        cg = []
        for axis, indices in enumerate(cg_indexes):
            axis_total = 0
            for cgpos, offsets in leg_cg_data:
                axis_total += sum(
                    (cgpos[idx] + offsets[axis]) * weight
                    for idx, weight in zip(indices, weights)
                )
            cg.append((axis_total + body_cg[axis] * Spot.Weight_Body) / weight_sum)

        return tuple(cg)


    def CG_distance (self,x_legs,y_legs,z_legs,xcg,ycg,stance):

        #line equation c * x + s * y - p  = 0
        # with c = a/m et s = b/m

        a1 = (y_legs[0]-y_legs[2])
        b1 = -(x_legs[0]-x_legs[2])
        m1 =sqrt(a1**2 + b1**2)
        c1 = a1/m1
        s1 = b1/m1

        a2 = (y_legs[1]-y_legs[3])
        b2 = -(x_legs[1]-x_legs[3])
        m2 =sqrt(a2**2 + b2**2)
        c2 = a2/m2
        s2 = b2/m2

        p1 = c1*x_legs[0] + s1*y_legs[0]
        p2 = c2*x_legs[1] + s2*y_legs[1]

        """ Dstance calculation """
        d1 = c1*xcg + s1*ycg - p1
        d2 = c2*xcg + s2*ycg - p2

        """ intersection calculation """
        #perpendicalar line equation -s * x + c * y - q = 0

        q1 = -s1*xcg +c1*ycg
        q2 = -s2*xcg +c2*ycg

        xint1 = c1*p1 - s1*q1
        yint1 = c1*q1 + s1*p1

        xint2 = c2*p2 - s2*q2
        yint2 = c2*q2 + s2*p2

        """ Check if inside sustentation triangle """
        d = 0
        xint = xcg
        yint = ycg
        if (stance[0] == False) or (stance[2] == False):
            d = d2
            xint = xint2
            yint = yint2


        if (stance[1] == False) or (stance[3] == False):
            d = d1
            xint = xint1
            yint = yint1

        balance = True

        if (stance[0] == False) and (d < 0):
            balance = False

        if (stance[1] == False) and (d > 0):
            balance = False

        if (stance[2] == False) and (d > 0):
            balance = False

        if (stance[3] == False) and (d < 0):
            balance = False

        d = abs(d) if balance else -abs(d)

        return (d,xint,yint,balance)
