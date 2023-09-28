clc;
clear all;
close all;

%Abrimos el fichero nivel de antena
nivel_antena=importdata('NIVEL_ANTENA.S2P');
nivel_antena=nivel_antena.data(:,4);

%Abrimos el fichero nivel de bocina patrón
nivel_bocina=importdata('NIVEL_BOCINA.S2P');
nivel_bocina=nivel_bocina.data(:,4);

%Abrimos el archivo con la ganancia de la bocina del fabricante
filename = 'FD_A4_4_.5';
delimiterIn = ',';
headerlinesIn = 13;
A = importdata(filename,delimiterIn,headerlinesIn);

Gboc=A.data;     %Gboc es la ganancia de la bocina patrón de 3.95Ghz a 5.8GHz
Gboc=Gboc(:,1);
x1=3.95:0.00925:5.8;
x2=4.5:0.005:5.5;
bocina_fabricante=interp1(x1, Gboc, x2);
bocina_fabricante=bocina_fabricante.';
Gtotal=nivel_antena-nivel_bocina+bocina_fabricante;

%Representamos la ganancia final
plot(x2,Gtotal);
title('Ganancia realizada de la antena')
xlabel('Frecuencia (GHz)')
ylabel('Ganancia (dB)')