%% Para ver los resultados, primero ejecutar esta sección y luego ejecutar
%  la frecuencia deseada. Para ejecutar cada sección, hacer click en 
%  "Run Section"
clc;
close all;
clear all;
filename = 'DIAGRAMA.dat';
delimiterIn = ' ';
headerlinesIn = 2;
A = importdata(filename,delimiterIn,headerlinesIn);
datos=A.data;

%% Diagrama de radiación normalizado a 4.8GHz
rad=[datos(:, 1),abs(datos(:, 123))];   %4.8 es 123 y 4.5 es 3
maximo=max(rad(:, 2));
plot(rad(:, 1),rad(:, 2)-maximo);
hold on;
B = importdata('4_8.txt');
B=B.data;
inv=flip(B(:,6));
plot(B(:,1),inv);
ylim([-30 0]);
xlim([-90 90]);
title('Diagrama de radiación normalizado a 4.8GHz')
xlabel('Ángulo acimutal (grados)')
ylabel('Magnitud normalizada (dB)')
legend('Diagrama rad. medido','Diagram rad. simulado')

%% Diagrama de radiación normalizado a 4.9GHz (frecuencia inferior del parche)
rad=[datos(:, 1),abs(datos(:, 163))];  
maximo=max(rad(:, 2));
plot(rad(:, 1),rad(:, 2)-maximo);
hold on;
B = importdata('4_9.txt');
B=B.data;
inv=flip(B(:,6));
plot(B(:,1),inv);
ylim([-30 0]);
xlim([-90 90]);
title('Diagrama de radiación normalizado a 4.9GHz')
xlabel('Ángulo acimutal (grados)')
ylabel('Magnitud normalizada (dB)')
legend('Diagrama rad. medido','Diagram rad. simulado')

%% Diagrama de radiación normalizado a 5GHz
rad=[datos(:, 1),abs(datos(:, 203))];
maximo=max(rad(:, 2));
plot(rad(:, 1),rad(:, 2)-maximo);
hold on;
B = importdata('5.txt');
B=B.data;
inv=flip(B(:,6));
plot(B(:,1),inv);
ylim([-30 0]);
xlim([-90 90]);
title('Diagrama de radiación normalizado a 5GHz')
xlabel('Ángulo acimutal (grados)')
ylabel('Magnitud normalizada (dB)')
legend('Diagrama rad. medido','Diagram rad. simulado')

%% Diagrama de radiación normalizado a 5.1GHz (frecuencia inferior del parche)
rad=[datos(:, 1),abs(datos(:, 243))];  
maximo=max(rad(:, 2));
plot(rad(:, 1),rad(:, 2)-maximo);
hold on;
B = importdata('5_1.txt');
B=B.data;
inv=flip(B(:,6));
plot(B(:,1),inv);
ylim([-30 0]);
xlim([-90 90]);
title('Diagrama de radiación normalizado a 5.1GHz')
xlabel('Ángulo acimutal (grados)')
ylabel('Magnitud normalizada (dB)')
legend('Diagrama rad. medido','Diagram rad. simulado')

%% Diagrama de radiación normalizado a 5.2GHz
rad=[datos(:, 1),abs(datos(:, 283))];  %5.2 es 283 y 5.46 es 387
maximo=max(rad(:, 2));
plot(rad(:, 1),rad(:, 2)-maximo);
hold on;
B = importdata('5_2.txt');
B=B.data;
inv=flip(B(:,6));
plot(B(:,1),inv);
ylim([-30 0]);
xlim([-90 90]);
title('Diagrama de radiación normalizado a 5.2GHz')
xlabel('Ángulo acimutal (grados)')
ylabel('Magnitud normalizada (dB)')
legend('Diagrama rad. medido','Diagram rad. simulado')