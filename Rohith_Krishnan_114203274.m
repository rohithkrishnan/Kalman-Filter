
%----------ENTS 659M (Intelligent Wireless Technologies Project) -----------
%Name: Rohith Prabha Krishnan
%UID: 114203274 
%Here we design a Kalman filter which is used to predict the distance and
%velocity of a moving vehicle. The prediction is done using the measurement
%values from the accelerometer updates and measurement is done with the
%help of GPS information made available.

%Clear all variables in the cache.
clc
clear

%Open the file containing the accelerometer readings and read into an
% array --> Driving_Force
fileID1 = fopen('driving_force.txt');
Driving_Force = textscan(fileID1,'%f');
fclose(fileID1);

%Open the file containing the GPS readings and read into an
% array --> GPS_data_unsorted
%We need only the time, position and velocity from the GPS data. These data
%can be safely extracted using the format specifier as given below.
fileID2 = fopen('GPS_measurement_data.txt','r');
GPS_data_unsorted = textscan(fileID2,'%f\n%*s%*s%f%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s\n%*s%*s%*s%*s%*s%*s%*s%f%*s','Delimiter',',');
fclose(fileID1);


%Initialisation of Variables
X{1,1} = [0;0]; %State Matrix
W0 = [0.1;0.001]; % Process Noise vector
B0 = [2.75*10^-10;0.1]; % Measurement noise vector
Q = [10 0;0 1]; % Process Noise covariance matrix
R = [2.75*10^-10 0;0 0.1]; % Measurement Noise covariance matrix
P{1,1} = Q;
F = [1 0.1;0 1];
I = eye(2); % Identity Matrix

%Setting up variables which will keep track of the time expired.
time_counter = 0;

% Extracting the GPS values*(Time, Distance, Velocity) into a column vector
 acc_reading = Driving_Force{1,1};
 GPS_time_stamp=GPS_data_unsorted{1,1};
 GPS_position = GPS_data_unsorted{1,2};
 GPS_velocity = GPS_data_unsorted{1,3}/3.6;

%Calculate the value of R-LAT
a = 6378137;
b = 6356752.3142;
f = (a-b)/a;
e = sqrt(f*(1-f));
r_lat = (a *(1-e^2)*2*pi)/(360*(1-(e^2*((sind(GPS_position(1,1)))^2)))^1.5);
H = [1/(r_lat+54) 0;0 3.6];

for i=1:1:length(GPS_position)
    GPS_meas_distance{i,1}(1,1)=(GPS_position(i,1) - GPS_position(1,1))*(r_lat+54);
    GPS_meas_distance{i,1}(2,1)=GPS_velocity(i,1);
end
j = 1;  %2
%run a loop for predicting the next values for Distance and Velocity
for i=2:1:length(acc_reading)
    %Every loop corresponds to the sampling interval of the accelerometer
    %i.e. 100 ms.
    time_counter = time_counter+0.1;
    X{i,1} = F*X{i-1,1}+[0;0.1]*acc_reading(i-1,1) + I*W0;
    P{i,1} = F*P{i-1,1}*F' + Q;
    %The following vector saves the prediction values alone without the
    %measurement update
    X_pred_pos(i,1) = X {i,1}(1,1);
    X_pred_vel(i,1) = X {i,1}(2,1); 
    %Measurement is done only inside the GPS block. Initialise all other
    %values with Nan so that they are not plotted.
    X_mes_pos(i,1) = NaN;
    X_mes_vel(i,1) = NaN;
    
    %The following loop will run until the GPS time stamp array reaches its
    %end
    if(j<=length(GPS_time_stamp))
        %GPS values can be only used after the current time reaches the GPS Time Stamp
        if (GPS_time_stamp(j,1) <= time_counter) 
            %disp(j);
            % calculating Kalman Gain
            K{j,1}= (P{i,1}*H')/((H*P{i,1}*H'+ R));
            Z{j,1} = H*GPS_meas_distance{j,1} + I*B0 ;
            X{i,1} = X{i,1} + K{j,1}*(Z{j,1}-H*X{i,1});
            P{i,1} = (I- (K{j,1}*H))*P{i,1};
            %Increment the count of the GPS Time Stamp counter
            j = j+1;
            % store the measurement values in order to plot them
            X_mes_pos(i,1) = X{i,1}(1,1);
            X_mes_vel(i,1) =X{i,1}(2,1);
        end
    end
end

time = 0:0.1:300;

%----PLOT----%
figure (1)
subplot(2,1,1)
% plotting predicted position and measured position with respect to time
% Blue dot represents Mesaured position updates from GPS and red dots
% represent the Predicted position updates.
plot(time,X_mes_pos,'b.',time,X_pred_pos,'r.')
legend('Measured Position','Predicted Position','Location','southeast')
xlabel('Time(s)')
ylabel('Distance(m)')
title('Distance Vs Time') 
subplot(2,1,2)
plot(time,X_mes_vel,'r.',time,X_pred_vel,'c.')
legend('Measured Velocity','Predicted Velocity','Location','northeast')
xlabel('Time(s)')
ylabel('Velocity(m/s)')
title('Velocity Vs Time')

