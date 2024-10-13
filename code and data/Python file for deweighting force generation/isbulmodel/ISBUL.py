# -*- coding: utf-8 -*-
"""
Created on Thu May  2 18:22:41 2024

Functionalities using the Robotics toolbox to define an ISb compatible
upper-limb model as a serial manipulator. Same joint angles representation as
the OpenSIM MoBL-ARMS model (https://simtk.org/projects/upexdyn).

@author: vcrocher - Unimelb
"""
import numpy as np
from spatialmath import SO3, SE3
import roboticstoolbox as rtb



def ISB7DofUL(ua_l: float, fa_l: float, ha_l: float, m_ua: float = 0, m_fa: float = 0) -> rtb.Robot:
    """
    ISB7DOFUL Create a Robot of robotic toolbox ISB compatible arm w/ shoulder elbow and wrist
    ua_l: upper-arm length
    fa_l: forearm length
    ha_l: hand length
    m_ua and m_fa: upper-arm and forearm masses, centered in middle of segment
    """

    L = [] #Links list
    #DH parameters definition (only revolute modified DH)
    L.append(rtb.RevoluteMDH(d=0.0,     a=0.0,  alpha=0.0,      offset=0,               qlim=[-np.pi/2, 130/180*np.pi],                                 name='PlaneEle')) # Planeele
    L.append(rtb.RevoluteMDH(d=0,       a=0.0,  alpha=np.pi/2,  offset=0,               qlim=[0, np.pi],                                                name='Elevation')) # Elevation
    L.append(rtb.RevoluteMDH(d=ua_l,    a=0.0,  alpha=np.pi/2,  offset=np.pi,           qlim=[-np.pi/2, 20/180*np.pi],     m = m_ua, r = [0,0,-ua_l/2], name='IntExtRot')) # Int/ext
    L.append(rtb.RevoluteMDH(d=0,       a=0.0,  alpha=np.pi/2,  offset=np.pi,           qlim=[0, 130/180*np.pi],                                        name='Elbow')) # Elbow flex
    L.append(rtb.RevoluteMDH(d=fa_l,    a=0.0,  alpha=np.pi/2,  offset=0,               qlim=[-np.pi/2, np.pi/2],          m = m_fa, r = [0,0,-fa_l/2], name='Pronosupination')) # Pronosupination
    L.append(rtb.RevoluteMDH(d=0,       a=0.0,  alpha=np.pi/2,  offset=np.pi/2,         qlim=[-20/180*np.pi, 20/180*np.pi],                             name='WristDev')) # Wrist deviation
    L.append(rtb.RevoluteMDH(d=0,       a=0.0,  alpha=np.pi/2,  offset=np.pi/2,         qlim=[-20/180*np.pi, np.pi/2],                                  name='WristFlex')) # Wrist flexion

    ISBUL = rtb.DHRobot(L)

    #Add hand transformation (tool) to match OpenSIM model wrist offset
    #frame: z -> x, x -> -y, y -> -z
    ISBUL.tool=SE3([[0,0,1,0],[-1,0,0,-ha_l],[0,-1,0,0],[0,0,0,1]]);

    return ISBUL



def ISB5DofUL(ua_l: float, fa_l: float, ha_l: float, m_ua: float = 0, m_fa: float = 0) -> rtb.Robot:
    """
    ISB5DOFUL Create a Robot of robotic toolbox ISB compatible arm w/ shoulder and elbow
    ua_l: upper-arm length
    fa_l: forearm length
    ha_l: hand length
    m_ua and m_fa: upper-arm and forearm masses, centered in middle of segment
    """

    L = [] #Links list
    #DH parameters definition (only revolute modified DH)
    L.append(rtb.RevoluteMDH(d=0.0,     a=0.0,  alpha=0.0,      offset=0,               qlim=[-np.pi/2, 130/180*np.pi],                                 name='PlaneEle')) # Planeele
    L.append(rtb.RevoluteMDH(d=0,       a=0.0,  alpha=np.pi/2,  offset=0,               qlim=[0, np.pi],                                                name='Elevation')) # Elevation
    L.append(rtb.RevoluteMDH(d=ua_l,    a=0.0,  alpha=np.pi/2,  offset=np.pi,           qlim=[-np.pi/2, 20/180*np.pi],     m = m_ua, r = [0,0,-ua_l/2], name='IntExtRot')) # Int/ext
    L.append(rtb.RevoluteMDH(d=0,       a=0.0,  alpha=np.pi/2,  offset=np.pi,           qlim=[0, 130/180*np.pi],                                        name='Elbow')) # Elbow flex
    L.append(rtb.RevoluteMDH(d=fa_l,    a=0.0,  alpha=np.pi/2,  offset=0,               qlim=[-np.pi/2, np.pi/2],          m = m_fa, r = [0,0,-fa_l/2], name='Pronosupination')) # Pronosupination

    ISBUL = rtb.DHRobot(L)

    #Add hand transformation (tool) to match OpenSIM model wrist offset
    #frame: z -> x, x -> -y, y -> -z
    ISBUL.tool=SE3([[0,0,1,0],[-1,0,0,-ha_l],[0,-1,0,0],[0,0,0,1]]);

    return ISBUL
