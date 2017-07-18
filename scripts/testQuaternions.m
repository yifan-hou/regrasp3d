clear;clc;

q0 = [1 0 0 0]';
a = [0 1 0]';
b = [-1 0 0]';
c = [0 0 1]';

N = 20;
ang = [0:(N-1)]/(N-1);



%% rot by a
anga = ang*pi/2;
% rotations
qa = zeros(4, N); 
% results
q01 = zeros(4, N);

for i = 1:N
	qa(:,i) = [cos(anga(i)/2); sin(anga(i)/2)*a];
end


for i = 1:N
	q01(:,i) = quatMTimes(qa(:,i), q0);
end
a = quatOnVec(a, qa(:,end));
b = quatOnVec(b, qa(:,end));
c = quatOnVec(c, qa(:,end));




%% rot by b
angb = ang*pi/2;
% rotations
qb = zeros(4, N); 
% results
q02 = zeros(4, N, N);

for i = 1:N
	qb(:,i) = [cos(angb(i)/2); sin(angb(i)/2)*b];
end


for i = 1:N
	for j = 1:N
		q02(:,i,j) = quatMTimes(qb(:,i), q01(:,j));
	end
end
q02 = reshape(q02, [4, N^2]);

a = quatOnVec(a, qb(:,end));
b = quatOnVec(b, qb(:,end));
c = quatOnVec(c, qb(:,end));


%% rot by c
angc = ang*pi/2;
% rotations
qc = zeros(4, N); 
% results
q03 = zeros(4, N^2, N);

for i = 1:N
	qc(:,i) = [cos(angc(i)/2); sin(angc(i)/2)*c];
end


for i = 1:N^2
	for j = 1:N
		q03(:,i,j) = quatMTimes(qc(:,j), q02(:,i));
	end
end

q03 = reshape(q03, [4, N^3]);


figure(1);clf;hold on;
plot3(q0(2), q0(3), q0(4), '*');
plot3(q01(2,:), q01(3,:), q01(4,:), '.', 'markersize', 8);
plot3(q02(2,:), q02(3,:), q02(4,:), '.', 'markersize', 8);
plot3(q03(2,:), q03(3,:), q03(4,:), '.', 'markersize', 8);
xlabel('x');ylabel('y');zlabel('z');

