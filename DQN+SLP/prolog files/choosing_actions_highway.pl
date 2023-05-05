% Define the positions and velocities of the ego car and the neighboring cars
% The format is vehicle(Name, Lane, Px, Py, Vx, Vy, TTLC)

:-dynamic vehicle/9.

:- ['vehicle_clauses.pl'].

% vehicle(ego, 4, 20, 4.5, 70, 0, 0).
% vehicle(v1, 4, 30, 4.5, 90, 0, 0).
% vehicle(v2, 6, 30, 4.5, 90, 0, 0).

% vehicle(v1, 1, 20, 1.5, 0, 0, 0).
% vehicle(v2, 3, 20, 7.5, 0, 0, 0).
% vehicle(v3, 1, 40, 1.5, 0, 0, 0).
% vehicle(v4, 2, 40, 4.5, 0, 0, 0).
% vehicle(v5, 1, 60, 1.5, 0, 0, 0).
% vehicle(v6, 3, 60, 7.5, 0, 0, 0).
% vehicle(v7, 2, 80, 4.5, 0, 0, 0).
% vehicle(v8, 3, 80, 7.5, 0, 0, 0).
% vehicle(v9, 1, 100, 1.5, 0, 0, 0).
% vehicle(v10,2, 100, 4.5, 0, 0, 0).

lane(Car,Lane):-vehicle(Car,Lane,_,_,_,_,_,_,_).

position_x(Car,Px):-vehicle(Car,_,Px,_,_,_,_,_,_).
position_y(Car,Py):-vehicle(Car,_,_,Py,_,_,_,_,_).

width(Car,W):-vehicle(Car,_,_,_,W,_,_,_,_).
height(Car,H):-vehicle(Car,_,_,_,_,H,_,_,_).

velocity_x(Car,Vx):-vehicle(Car,_,_,_,_,_,Vx,_,_).
velocity_y(Car,Vy):-vehicle(Car,_,_,_,_,_,_,Vy,_).

% Define the maximum speed limit and the safe following distance
min_speed(20).
max_speed(Lane,MaxSpeed):-(Lane is 1 -> MaxSpeed is 100;
                           Lane is 2 -> MaxSpeed is 110;
                           Lane is 3 -> MaxSpeed is 120;
                           Lane is 4 -> MaxSpeed is 120;
                           Lane is 5 -> MaxSpeed is 110;
                           Lane is 6 -> MaxSpeed is 100).

time_step(0.04).
critical_distance(30).
safe_distance(X):-critical_distance(Y),X>Y.

% :-vehicle(ego,Lane,_,_,Vx,_,_),max_speed(Lane,MaxSpeed),
%   ((Vx=<MaxSpeed)->writeln("Speed is valid.");writeln("Warning: Ego speed is out of bound. Please reduce it.")).

% Define the available actions for the ego car
% The format is action(ActionName)
action(right_lane_change).
action(left_lane_change).
action(lane_keeping).

% How much is the distance between Car1 and Car2?
distance(Car1,Car2,D):-vehicle(Car1,_,X1,Y1,_,_,_,_,_),
                        vehicle(Car2,_,X2,Y2,_,_,_,_,_), 
                        D is ((X1-X2)^2+(Y1-Y2)^2)^0.5.

distanceX(Car1,Car2,D):-vehicle(Car1,_,X1,_,W1,_,_,_,_),
                        vehicle(Car2,_,X2,_,W2,_,_,_,_), 
                        (abs(X1-X2)>0.5*(W1+W2) -> D is abs(X1-X2)-0.5*(W1+W2); D is 0).

distanceY(Car1,Car2,D):-vehicle(Car1,_,_,Y1,_,H1,_,_,_),
                        vehicle(Car2,_,_,Y2,_,H2,_,_,_), 
                        D is abs(Y1-Y2)-0.5*(H1+H2).

relativeVelocity(Car1,Car2,RelVel):-vehicle(Car1,_,_,_,_,_,Vx1,_,_),
                                    vehicle(Car2,_,_,_,_,_,Vx2,_,_),
                                    RelVel is (Vx2-Vx1).

% Adjacent cars
neighboring_cars(Car):-distance(ego, Car, D), D =< 70, Car \= ego.
%neighboring_cars2(Cars):-findall(Car, (distance(ego,Car,D),D =< 30, Car \= ego), Cars).

% Relative positions of adjacent cars
north(Car):-neighboring_cars(Car),
            vehicle(ego,Lane1,Px1,_,_,_,_,_,_),
            vehicle(Car,Lane2,Px2,_,_,_,_,_,_),
            Lane1 is Lane2,
            (Lane1<4,Px2<Px1;Lane1>=4,Px2>Px1),!.

south(Car):-neighboring_cars(Car),
            vehicle(ego,Lane1,Px1,_,_,_,_,_,_),
            vehicle(Car,Lane2,Px2,_,_,_,_,_,_),
            Lane1 is Lane2,
            (Lane1<4,Px2>Px1;Lane1>=4,Px2<Px1),!.

east(Car):-neighboring_cars(Car),
            vehicle(ego,Lane1,_,_,_,_,_,_,_),
            vehicle(Car,Lane2,_,_,_,_,_,_,_),
            distanceX(ego,Car,D),D =< 10,
            (Lane1<4,Lane2 is Lane1-1;Lane1>=4,Lane2 is Lane1+1),!.

west(Car):-neighboring_cars(Car),
            vehicle(ego,Lane1,_,_,_,_,_,_,_),
            vehicle(Car,Lane2,_,_,_,_,_,_,_),
            distanceX(ego,Car,D),D =< 10,
            (Lane1<4,Lane2 is Lane1+1;Lane1>=4,Lane2 is Lane1-1),!.

northeast(Car):-neighboring_cars(Car),
                vehicle(ego,Lane1,Px1,_,_,_,_,_,_),
                vehicle(Car,Lane2,Px2,_,_,_,_,_,_),
                distanceX(ego,Car,D),D > 10,
                (Lane1<4,Lane2 is Lane1-1,Px2 < Px1;Lane1>=4,Lane2 is Lane1+1,Px2 > Px1),!.

northwest(Car):-neighboring_cars(Car),
                vehicle(ego,Lane1,Px1,_,_,_,_,_,_),
                vehicle(Car,Lane2,Px2,_,_,_,_,_,_),
                distanceX(ego,Car,D),D > 10,
                (Lane1<4,Lane2 is Lane1+1,Px2 < Px1;Lane1>=4,Lane2 is Lane1-1,Px2 > Px1),!.

southeast(Car):-neighboring_cars(Car),
                vehicle(ego,Lane1,Px1,_,_,_,_,_,_),
                vehicle(Car,Lane2,Px2,_,_,_,_,_,_),
                distanceX(ego,Car,D),D > 10,
                (Lane1<4,Lane2 is Lane1-1,Px2 > Px1;Lane1>=4,Lane2 is Lane1+1,Px2 < Px1),!.

southwest(Car):-neighboring_cars(Car),
                vehicle(ego,Lane1,Px1,_,_,_,_,_,_),
                vehicle(Car,Lane2,Px2,_,_,_,_,_,_),
                distanceX(ego,Car,D),D > 10,
                (Lane1<4,Lane2 is Lane1+1,Px2 > Px1;Lane1>=4,Lane2 is Lane1-1,Px2 < Px1),!.

% Is there any car near the ego car?
% :-north(Car),write("vehicle "),write(Car),writeln(" is in front.").
% :-northeast(Car),write("vehicle "),write(Car),writeln(" is in front-right.").
% :-northwest(Car),write("vehicle "),write(Car),writeln(" is in front-left.").
% :-south(Car),write("vehicle "),write(Car),writeln(" is in back.").
% :-southeast(Car),write("vehicle "),write(Car),writeln(" is in back-right.").
% :-southwest(Car),write("vehicle "),write(Car),writeln(" is in back-left.").
% :-east(Car),write("vehicle "),write(Car),writeln(" is in right.").
% :-west(Car),write("vehicle "),write(Car),writeln(" is in left.").

% Busy sections relative to the ego car:
north_is_busy:-neighboring_cars(Car),
                vehicle(ego,Lane1,Px1,_,_,_,_,_,_),
                vehicle(Car,Lane2,Px2,_,_,_,_,_,_),
                Lane1 is Lane2,
                (Lane1<4,Px2 < Px1;Lane1>=4,Px2 > Px1),!.

northeast_is_busy:-neighboring_cars(Car),
                vehicle(ego,Lane1,Px1,_,_,_,_,_,_),
                vehicle(Car,Lane2,Px2,_,_,_,_,_,_),
                distanceX(ego,Car,D),D > 0,
                (Lane1<4,Lane2 is Lane1-1,Px2 < Px1;Lane1>=4,Lane2 is Lane1+1,Px2 > Px1),!.

northwest_is_busy:-neighboring_cars(Car),
                vehicle(ego,Lane1,Px1,_,_,_,_,_,_),
                vehicle(Car,Lane2,Px2,_,_,_,_,_,_),
                distanceX(ego,Car,D),D > 0,
                (Lane1<4,Lane2 is Lane1+1,Px2 < Px1;Lane1>=4,Lane2 is Lane1-1,Px2 > Px1),!.

east_is_busy:-neighboring_cars(Car),
                vehicle(ego,Lane1,_,_,_,_,_,_,_),
                vehicle(Car,Lane2,_,_,_,_,_,_,_),
                distanceX(ego,Car,D),D is 0,
                (Lane1<4,Lane2 is Lane1-1;Lane1>=4,Lane2 is Lane1+1),!.

west_is_busy:-neighboring_cars(Car),
                vehicle(ego,Lane1,_,_,_,_,_,_,_),
                vehicle(Car,Lane2,_,_,_,_,_,_,_),
                (Lane1<4,Lane2 is Lane1+1;Lane1>=4,Lane2 is Lane1-1),
                distanceX(ego,Car,D),D is 0,!.

south_is_busy:-neighboring_cars(Car),
                vehicle(ego,Lane1,Px1,_,_,_,_,_,_),
                vehicle(Car,Lane2,Px2,_,_,_,_,_,_),
                Lane2 is Lane1,
                (Lane1<4,Px2 > Px1;Lane1>=4,Px2 < Px1),
                relativeVelocity(ego,Car,RelVel),
                RelVel > 0,!.

southeast_is_busy:-neighboring_cars(Car),
                vehicle(ego,Lane1,Px1,_,_,_,_,_,_),
                vehicle(Car,Lane2,Px2,_,_,_,_,_,_),
                distanceX(ego,Car,D),D > 0,
                (Lane1<4,Lane2 is Lane1-1,Px2 > Px1;Lane1>=4,Lane2 is Lane1+1,Px2 < Px1),
                relativeVelocity(ego,Car,RelVel),
                RelVel > 0,!.

southwest_is_busy:-neighboring_cars(Car),
                vehicle(ego,Lane1,Px1,_,_,_,_,_,_),
                vehicle(Car,Lane2,Px2,_,_,_,_,_,_),
                distanceX(ego,Car,D),D > 0,
                (Lane1<4,Lane2 is Lane1+1,Px2 > Px1;Lane1>=4,Lane2 is Lane1-1,Px2 < Px1),
                relativeVelocity(ego,Car,RelVel),
                RelVel > 0,!.

% Free sections relative to the ego car:
north_is_free:-not(north_is_busy).
northeast_is_free:-not(northeast_is_busy).
northwest_is_free:-not(northwest_is_busy).
east_is_free:-not(east_is_busy).
west_is_free:-not(west_is_busy).
south_is_free:-not(north_is_busy).
southeast_is_free:-not(northeast_is_busy).
southwest_is_free:-not(northwest_is_busy).

% ================================================
% Which action is safe?
safe_actions(Action):-vehicle(ego,Lane,_,_,_,_,_,_,_),
                    not(Lane is 3),not(Lane is 4),
                    (north_is_busy;south_is_busy),
                    west_is_free,
                    northwest_is_free,
                    Action = left_lane_change,!.

safe_actions(Action):-vehicle(ego,Lane,_,_,_,_,_,_,_),
                    not(Lane is 1),not(Lane is 6),
                    (north_is_busy;south_is_busy),
                    east_is_free,
                    northeast_is_free,
                    Action = right_lane_change,!.

safe_actions(Action):-Action = lane_keeping.

% possible actions
safe_actions1(Action):-Action = lane_keeping.

safe_actions1(Action):-vehicle(ego,Lane,_,_,_,_,_,_,_),
                    not(Lane is 3),not(Lane is 4),
                    west_is_free, northwest_is_free, southwest_is_free,
                    Action = left_lane_change.

safe_actions1(Action):-vehicle(ego,Lane,_,_,_,_,_,_,_),
                    not(Lane is 1),not(Lane is 6),
                    east_is_free, northeast_is_free, southeast_is_free,
                    Action = right_lane_change.

possible_actions(Actions):-findall(Action,safe_actions1(Action),Actions).

% ====================================================
% What is the best velocity in each state?
desired_velocity_x(V_x):-north_is_free,
                        vehicle(ego,Lane,_,_,_,_,_,_,_),
                        max_speed(Lane,X),
                        V_x is X,!.

desired_velocity_x(V_x):-north_is_busy,
                        vehicle(ego,Lane,_,_,_,_,Vx1,_,_),
                        vehicle(Car,_,_,_,_,_,Vx2,_,_),
                        distance(ego,Car,D),
                        safe_distance(D),
                        critical_distance(C),
                        A is (Vx2^2-Vx1^2)/2/(D-C),
                        % write(A),
                        time_step(Dt),
                        V is Vx1+A*Dt,
                        max_speed(Lane,MaxSpeed),
                        ((V>MaxSpeed) -> V_x is MaxSpeed;V_x is V),!.

desired_velocity_x(V_x):-north_is_busy,
                        vehicle(ego,Lane,_,_,_,_,Vx1,_,_),
                        vehicle(Car,_,_,_,_,_,_,_,_),
                        distance(ego,Car,D),
                        critical_distance(C),
                        D =< C,
                        ((D > 0) -> A is -Vx1^2/2/D; A is -5),
                        % write(A),
                        time_step(Dt),
                        V is Vx1+A*Dt,
                        max_speed(Lane,MaxSpeed),
                        ((V>MaxSpeed) -> V_x is MaxSpeed;V_x is V),
                        min_speed(MinSpeed),
                        ((V<MinSpeed) -> V_x is MinSpeed;V_x is V).

% rules for longitudinal acceleration
acceleration_x(Ax):-north_is_free,
                    vehicle(ego,Lane,_,_,_,_,Vego,_,_),
                    max_speed(Lane,Vmax),
                    time_step(Dt),
                    Ax is (Vmax-Vego)/Dt,!.

acceleration_x(Ax):-north_is_busy,
                    vehicle(ego,Lane1,_,_,_,_,Vx1,_,_),
                    vehicle(Car,Lane2,_,_,_,_,Vx2,_,_),
                    Lane1 is Lane2,
                    distanceX(ego,Car,D),
                    safe_distance(D),
                    critical_distance(C),
                    Ax is (Vx2^2-Vx1^2)/2/(D-C),!.

acceleration_x(Ax):-north_is_busy,
                    vehicle(ego,Lane1,_,_,_,_,Vx1,_,_),
                    vehicle(Car,Lane2,_,_,_,_,_,_,_),
                    Lane1 is Lane2,
                    distanceX(ego,Car,D),
                    critical_distance(C),
                    D =< C,
                    ((D > 0) -> Ax is -Vx1^2/2/D; Ax is -5).