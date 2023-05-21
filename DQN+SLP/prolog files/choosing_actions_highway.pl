% Define the positions and velocities of the ego car and the neighboring cars
% The format is vehicle(Name, Lane, Px, Py, Vx, Vy, TTLC)

:-dynamic vehicle/8.

:- ['vehicle_clauses.pl'].  % Load vehicles info from a file

% =============================================================================================
lane(Car,Lane):-vehicle(Car,Lane,_,_,_,_,_,_).

position_x(Car,Px):-vehicle(Car,_,Px,_,_,_,_,_).
position_y(Car,Py):-vehicle(Car,_,_,Py,_,_,_,_).

width(Car,W):-vehicle(Car,_,_,_,W,_,_,_).
height(Car,H):-vehicle(Car,_,_,_,_,H,_,_).

velocity_x(Car,Vx):-vehicle(Car,_,_,_,_,_,Vx,_).
velocity_y(Car,Vy):-vehicle(Car,_,_,_,_,_,_,Vy).

% =============================================================================================
% Define the maximum speed limit and the safe following distance
min_speed(20).
max_speed(120).
max_lane(6).
max_speed(Lane,MaxSpeed):-(Lane is 1 -> MaxSpeed is 100;
                           Lane is 2 -> MaxSpeed is 110;
                           Lane is 3 -> MaxSpeed is 120;
                           Lane is 4 -> MaxSpeed is 120;
                           Lane is 5 -> MaxSpeed is 110;
                           Lane is 6 -> MaxSpeed is 100;
                           Lane is 0 -> MaxSpeed is 0).

time_step(0.04).
critical_distance(20).
safe_distance(X):-critical_distance(Y),X>Y.
radar_range(70).

% Define the available actions for the ego car
% The format is action(ActionName)
action(lane_change).
action(right_lane_change).
action(left_lane_change).
action(lane_keeping).

% =============================================================================================
% How much is the distance between Car1 and Car2?
distanceX(Car1,Car2,D):-vehicle(Car1,_,X1,_,W1,_,_,_),
                        vehicle(Car2,_,X2,_,W2,_,_,_), 
                        (abs(X1-X2)>0.5*(W1+W2) -> D is abs(X1-X2)-0.5*(W1+W2); D is 0).

distanceY(Car1,Car2,D):-vehicle(Car1,_,_,Y1,_,H1,_,_),
                        vehicle(Car2,_,_,Y2,_,H2,_,_), 
                        (abs(Y1-Y2)>0.5*(H1+H2) -> D is abs(Y1-Y2)-0.5*(H1+H2); D is 0).

distance(Car1,Car2,D):-distanceX(Car1,Car2,Dx),
                        distanceY(Car1,Car2,Dy), 
                        D is (Dx^2+Dy^2)^0.5.

relativeVelocity(Car1,Car2,RelVel):-vehicle(Car1,_,_,_,_,_,Vx1,_),
                                    vehicle(Car2,_,_,_,_,_,Vx2,_),
                                    RelVel is (Vx2-Vx1).

% Adjacent cars
neighboring_cars(Car):-distance(ego, Car, D), radar_range(R), D =< R, Car \= ego.
%neighboring_cars2(Cars):-findall(Car, (distance(ego,Car,D),D =< 30, Car \= ego), Cars).

% =============================================================================================
% Relative positions of adjacent cars
north(Car):-neighboring_cars(Car),
            vehicle(ego,Lane1,Px1,_,_,_,_,_),
            vehicle(Car,Lane2,Px2,_,_,_,_,_),
            Lane1 is Lane2,
            (Lane1<4,Px2<Px1;Lane1>=4,Px2>Px1),
            distanceX(ego,Car,D),D>0,!.

south(Car):-neighboring_cars(Car),
            vehicle(ego,Lane1,Px1,_,_,_,_,_),
            vehicle(Car,Lane2,Px2,_,_,_,_,_),
            Lane1 is Lane2,
            (Lane1<4,Px2>Px1;Lane1>=4,Px2<Px1),
            distanceX(ego,Car,D),D>0,!.

east(Car):-neighboring_cars(Car),
            vehicle(ego,Lane1,_,_,_,_,_,_),
            vehicle(Car,Lane2,_,_,_,_,_,_),
            distanceX(ego,Car,D),D is 0,
            (Lane1<4,Lane2 is Lane1-1;Lane1>=4,Lane2 is Lane1+1),!.

west(Car):-neighboring_cars(Car),
            vehicle(ego,Lane1,_,_,_,_,_,_),
            vehicle(Car,Lane2,_,_,_,_,_,_),
            distanceX(ego,Car,D),D is 0,
            (Lane1<4,Lane2 is Lane1+1;Lane1>=4,Lane2 is Lane1-1),!.

northeast(Car):-neighboring_cars(Car),
                vehicle(ego,Lane1,Px1,_,_,_,_,_),
                vehicle(Car,Lane2,Px2,_,_,_,_,_),
                distanceX(ego,Car,D),D > 0,
                (Lane1<4,Lane2 is Lane1-1,Px2 < Px1;Lane1>=4,Lane2 is Lane1+1,Px2 > Px1),!.

northwest(Car):-neighboring_cars(Car),
                vehicle(ego,Lane1,Px1,_,_,_,_,_),
                vehicle(Car,Lane2,Px2,_,_,_,_,_),
                distanceX(ego,Car,D),D > 0,
                (Lane1<4,Lane2 is Lane1+1,Px2 < Px1;Lane1>=4,Lane2 is Lane1-1,Px2 > Px1),!.

southeast(Car):-neighboring_cars(Car),
                vehicle(ego,Lane1,Px1,_,_,_,_,_),
                vehicle(Car,Lane2,Px2,_,_,_,_,_),
                distanceX(ego,Car,D),D > 0,
                (Lane1<4,Lane2 is Lane1-1,Px2 > Px1;Lane1>=4,Lane2 is Lane1+1,Px2 < Px1),!.

southwest(Car):-neighboring_cars(Car),
                vehicle(ego,Lane1,Px1,_,_,_,_,_),
                vehicle(Car,Lane2,Px2,_,_,_,_,_),
                distanceX(ego,Car,D),D > 0,
                (Lane1<4,Lane2 is Lane1+1,Px2 > Px1;Lane1>=4,Lane2 is Lane1-1,Px2 < Px1),!.

% =============================================================================================
% Busy sections relative to the ego car:
north_is_busy:-neighboring_cars(Car),
                vehicle(ego,Lane1,Px1,_,_,_,_,_),
                vehicle(Car,Lane2,Px2,_,_,_,_,_),
                Lane1 is Lane2,
                (Lane1<4,Px2 < Px1;Lane1>=4,Px2 > Px1),
                distanceX(ego,Car,D),D>0,!.

northeast_is_busy:-neighboring_cars(Car),
                vehicle(ego,Lane1,Px1,_,_,_,_,_),
                vehicle(Car,Lane2,Px2,_,_,_,_,_),
                distanceX(ego,Car,D),D > 0,
                (Lane1<4,Lane2 is Lane1-1,Px2 < Px1;Lane1>=4,Lane2 is Lane1+1,Px2 > Px1),!.

northwest_is_busy:-neighboring_cars(Car),
                vehicle(ego,Lane1,Px1,_,_,_,_,_),
                vehicle(Car,Lane2,Px2,_,_,_,_,_),
                distanceX(ego,Car,D),D > 0,
                (Lane1<4,Lane2 is Lane1+1,Px2 < Px1;Lane1>=4,Lane2 is Lane1-1,Px2 > Px1),!.

east_is_busy:-neighboring_cars(Car),
                vehicle(ego,Lane1,_,_,_,_,_,_),
                vehicle(Car,Lane2,_,_,_,_,_,_),
                distanceX(ego,Car,D),D is 0,
                (Lane1<4,Lane2 is Lane1-1;Lane1>=4,Lane2 is Lane1+1),!.

west_is_busy:-neighboring_cars(Car),
                vehicle(ego,Lane1,_,_,_,_,_,_),
                vehicle(Car,Lane2,_,_,_,_,_,_),
                (Lane1<4,Lane2 is Lane1+1;Lane1>=4,Lane2 is Lane1-1),
                distanceX(ego,Car,D),D is 0,!.

south_is_busy:-neighboring_cars(Car),
                vehicle(ego,Lane1,Px1,_,_,_,_,_),
                vehicle(Car,Lane2,Px2,_,_,_,_,_),
                Lane2 is Lane1,
                (Lane1<4,Px2 > Px1;Lane1>=4,Px2 < Px1),
                distanceX(ego,Car,D),D>0,!.

southeast_is_busy:-neighboring_cars(Car),
                vehicle(ego,Lane1,Px1,_,_,_,_,_),
                vehicle(Car,Lane2,Px2,_,_,_,_,_),
                distanceX(ego,Car,D),D > 0,
                (Lane1<4,Lane2 is Lane1-1,Px2 > Px1;Lane1>=4,Lane2 is Lane1+1,Px2 < Px1),!.

southwest_is_busy:-neighboring_cars(Car),
                vehicle(ego,Lane1,Px1,_,_,_,_,_),
                vehicle(Car,Lane2,Px2,_,_,_,_,_),
                distanceX(ego,Car,D),D > 0,
                (Lane1<4,Lane2 is Lane1+1,Px2 > Px1;Lane1>=4,Lane2 is Lane1-1,Px2 < Px1),!.

ego_location_is_busy:-neighboring_cars(Car),
                vehicle(ego,Lane1,_,_,_,_,_,_),
                vehicle(Car,Lane2,_,_,_,_,_,_),
                Lane1 is Lane2,
                distanceX(ego,Car,D),D is 0,!.

% Free sections relative to the ego car:
north_is_free:-not(north_is_busy).
northeast_is_free:-not(northeast_is_busy).
northwest_is_free:-not(northwest_is_busy).
east_is_free:-not(east_is_busy).
west_is_free:-not(west_is_busy).
south_is_free:-not(south_is_busy).
southeast_is_free:-not(southeast_is_busy).
southwest_is_free:-not(southwest_is_busy).

% =============================================================================================
% possible actions
safe_actions(Action):-ego_location_is_busy,
                    (west_is_free->Action=left_lane_change;east_is_free->Action=right_lane_change;Action=lane_keeping),!.

safe_actions(Action):-
    ((((east_is_free;west_is_free),south_is_busy,south(Car),distanceX(ego,Car,D),critical_distance(C),D<C))->Action = lane_change;
    Action = lane_keeping).

safe_actions(Action):-vehicle(ego,Lane,_,_,_,_,_,_),
                    not(Lane is 3),not(Lane is 4),
                    (west_is_free,southwest_is_free,northwest_is_free;
                    west_is_free,northwest_is_free,southwest_is_busy,southwest(Car),relativeVelocity(ego,Car,RV),RV < 0),
                    Action = left_lane_change.

safe_actions(Action):-vehicle(ego,Lane,_,_,_,_,_,_),
                    not(Lane is 1),not(Lane is 6),
                    (east_is_free,southeast_is_free,northeast_is_free;
                    east_is_free,northeast_is_free,southeast_is_busy,southeast(Car),relativeVelocity(ego,Car,RV),RV < 0),
                    Action = right_lane_change.

possible_actions(Actions):-findall(Action,safe_actions(Action),Actions).

% =============================================================================================
% rules for longitudinal acceleration
acceleration_x(Ax):-(north_is_free;ego_location_is_busy),
                    vehicle(ego,Lane,_,_,_,_,Vego,_),
                    max_speed(Lane,Vmax),
                    time_step(Dt),
                    Ax is (Vmax-Vego)/Dt,!.

acceleration_x(Ax):-north_is_busy,
                    vehicle(ego,Lane1,_,_,_,_,Vx1,_),
                    vehicle(Car,Lane2,_,_,_,_,Vx2,_),
                    Lane1 is Lane2,
                    distanceX(ego,Car,D),
                    safe_distance(D),
                    critical_distance(C),
                    Eps is 0.1,
                    Ax is (Vx2^2-Vx1^2)/2/(abs(D-C)+Eps),!.

acceleration_x(Ax):-north_is_busy,
                    vehicle(ego,Lane1,_,_,_,_,Vx1,_),
                    vehicle(Car,Lane2,_,_,_,_,_,_),
                    Lane1 is Lane2,
                    distanceX(ego,Car,D),
                    critical_distance(C),
                    Eps is 0.1,
                    D =< C,Ax is -Vx1^2/2/(abs(D)+Eps),!.

longitudinal_velocity(Vx):-velocity_x(ego,V0),acceleration_x(Ax),time_step(Dt),V1 is V0+Ax*Dt,
                            min_speed(MinSpeed),max_speed(MaxSpeed),
                            (V1>MaxSpeed->Vx is MaxSpeed;V1<MinSpeed->Vx is MinSpeed;Vx is V1).

% =============================================================================================
% Extract the state list
state(A):-(north_is_free->A is 1;
           north_is_busy->north(Car),distanceX(ego,Car,D),radar_range(R),A is D/R;
           A is -1).
state(A):-(lane(ego,Lane),Lane is 6->A is 0;
           (northeast_is_free->A is 1;
            northeast_is_busy->northeast(Car),distance(ego,Car,D),radar_range(R),A is D/R;
            A is -1)).
state(A):-(lane(ego,Lane),Lane is 6->A is 0;
            (east_is_free->A is 1;
            east_is_busy->east(Car),distanceY(ego,Car,D),radar_range(R),A is D/R;
            A is -1)).
state(A):-(lane(ego,Lane),Lane is 6->A is 0;
            (southeast_is_free->A is 1;
            southeast_is_busy->southeast(Car),distance(ego,Car,D),radar_range(R),A is D/R;
            A is -1)).
state(A):-(south_is_free->A is 1;
            south_is_busy->south(Car),distanceX(ego,Car,D),radar_range(R),A is D/R;
            A is -1).
state(A):-(lane(ego,Lane),Lane is 4->A is 0;
            (southwest_is_free->A is 1;
            southwest_is_busy->southwest(Car),distance(ego,Car,D),radar_range(R),A is D/R;
            A is -1)).
state(A):-(lane(ego,Lane),Lane is 4->A is 0;
            (west_is_free->A is 1;
            west_is_busy->west(Car),distanceY(ego,Car,D),radar_range(R),A is D/R;
            A is -1)).
state(A):-(lane(ego,Lane),Lane is 4->A is 0;
            (northwest_is_free->A is 1;
            northwest_is_busy->northwest(Car),distance(ego,Car,D),radar_range(R),A is D/R;
            A is -1)).
state(A):-vehicle(ego,Lane,_,_,_,_,_,_),max_lane(MaxLane),A is Lane/MaxLane.
state(A):-vehicle(ego,_,_,_,_,_,Vx,_),max_speed(MaxSpeed),A is Vx/MaxSpeed.

states(States):-findall(State,state(State),States).