%CLASSASV   ASV object class for kinematic simulation and path tracking.
%
%   This class defines a simple ASV model for use in multi-vessel 
%   navigation, encounter, and path tracking simulations. Each ASV 
%   object stores position, velocity, course, rudder and other key 
%   properties, and provides methods to update state and trajectory.
%
%   Properties:
%     length      : ASV length (default: 40 m)
%     speed       : Current speed (m/s)
%     course      : Course angle (rad)
%     heading     : Heading angle (rad)
%     position    : [yEast, xNorth], position in NED coordinates (m)
%     yaw         : Yaw angle (rad)
%     yaw_rate    : Yaw rate (rad/s)
%     rudder      : Rudder angle (rad)
%     motorspeed  : Propeller/motor speed (user-defined, optional)
%     encounter   : Encounter situation indicator 
%                   (0: no risk, 1: head-on, 2: overtaking, 3: crossing)
%     path        : History of positions [n×2]
%     courses     : History of course angles [n×1]
%     speeds      : History of speed [n×1]
%
%   Methods:
%     ClassASV          : Constructor. Initializes with input vector.
%     ChangePosition     : Increment position by delta.
%     MoveToNewPosition  : Move to absolute position (optionally set course).
%     ChangeCourse       : Update course angle.
%     EncounterSituation : Set encounter situation code.
%
%   Usage:
%     ASV = ClassASV([speed, course, length, yEast, xNorth]);
%     ASV = ASV.ChangePosition([dy, dx]);
%     ASV = ASV.MoveToNewPosition([yNew, xNew, courseNew]);
%
%   Author: Zhibo He
%   Date:   2025-03-01
classdef ClassASV < handle
    % This is a class named ASV, only consists of speed
    % course, length, position
    properties
        length   = 1.5;
        speed    = -0;
        course   = -0;
        heading = 0;
        position = [0,0]; % NED, yEast,xNorth
        yaw = 0;
        yaw_rate = 0;
        rudder = 0;
        motorspeed = 0;
        path = [];
        courses = [];
        speeds = [];
    end

    methods
        function obj = ClassASV(InputMat)
            obj.speed    = InputMat(1);
            obj.course   = InputMat(2);
            obj.length   = InputMat(3);
            obj.position = InputMat(4:5);
            obj.path     = InputMat(4:5);
        end
        function obj = ChangePosition(obj,d_position)
            obj.position = obj.position + d_position;
            obj.path     = [obj.path;obj.position];
            obj.courses  = [obj.courses;obj.course];
            obj.speeds   = [obj.speeds;obj.speed];
        end
        function obj = MoveToNewPosition(obj,position)
            obj.position = position(1:2);
            if size(position,2) < 3
                
            else
                obj.course = position(3);
            end
            obj.path     = [obj.path;obj.position];
            obj.courses  = [obj.courses;obj.course];
            obj.speeds   = [obj.speeds;obj.speed];
        end
        function obj = ChangeCourse(obj,course)
            obj.course = course;
        end

    end
end
