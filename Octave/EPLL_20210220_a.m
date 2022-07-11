%%% Circuito de Sincronismo E_PLL %%%


%%%__Declaração de Variáveis:___%%%

Ts = 1e-5;            %Período total de simulação
Fs=1/Ts;              %Frequencia de amostragem [Hz]
T_sim = 0.2;          %Período total de simulação
L = ceil(T_sim/Ts);   % dividir em valores inteiros, apenas.
t=1:L;                %Periodo de amostragem

fm1=60;               %frequência fundamental
delta=pi/6;

%%%__Variáveis de Entrada

kp=30; ki=2000; ka = 200; wo=2*pi*51;

%variaveis internas

erro = zeros(1,L); 
sda_pll = zeros(1,L); sda_pll(1)=0;
v_in = zeros(1,L);

% malha de fase
erro_fase = zeros(1,L); erro_fase(1) = 0;
acum_w = zeros(1,L); acum_w(1) = 0;
w = zeros(1,L); w(1) = 0; wp = zeros(1,L); wi = zeros(1,L);
teta = zeros(1,L); teta(1) = 0;

% malha de amplitude
erro_amp = zeros(1,L);erro_amp(1) = 0;
amp = zeros(1,L); amp(1) = 0;

%variaveis de estado
cont = zeros(1,L); cont(1)=0;

for k =2:L

  tempo = t.*Ts;
  cont(k) = cont(k-1) + 1;
  x1 = 5*sin(2*pi*(fm1/Fs).*t);
  x2 = 5*sin(2*pi*(fm1/Fs).*t - (pi/2)) ;
  if (cont(k)<10001)
    v_in(k) = x1(k);
  else
    v_in(k) = x2(k);
  endif
  
  erro(k) = v_in(k)-sda_pll(k-1);
  
  
  % Malha de Fase do E-PLL
 
  erro_fase(k)= erro(k).*cos(teta(k-1));

  acum_w(k)= acum_w(k-1)+(erro_fase(k)+erro_fase(k-1))*ki*Ts/2;
  wp(k)= kp*erro_fase(k);
  wi(k)= wo + (acum_w(k));
  w(k)=wp(k)+wi(k);

  teta(k)= teta(k-1) + (w(k)+w(k-1))*Ts/2;

  if(teta(k) > (2*pi))
    teta(k) = 0;
  endif

% Malha de Amplitude do E-PLL modificado

  erro_amp(k) = erro(k).*sin(teta(k-1));
  amp(k) = amp(k-1) + (erro_amp(k)+erro_amp(k-1))*ka*Ts/2;
  
  sda_pll(k) = amp(k).*sin(teta(k));

endfor