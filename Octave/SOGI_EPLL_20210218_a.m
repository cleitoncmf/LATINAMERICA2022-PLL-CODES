%%% Circuito de Sincronismo SOGI E_PLL %%%


%%%__Declaração de Variáveis:___%%%

Ts = 1e-5;            %Período total de simulação
Fs=1/Ts;              %Frequencia de amostragem [Hz]
T_sim = 0.4;          %Período total de simulação
L = ceil(T_sim/Ts);   % dividir em valores inteiros, apenas.
t=1:L;                %Periodo de amostragem

fm1=60;               %frequência fundamental
fm2 = 50;
delta=pi/6;

%%%__Variáveis de Entrada

kp=10; ki=2000; K=2; Ka = 50; wo=2*pi*51;

v_in = zeros(1,L);
erro1_sogi = zeros(1,L); teta =zeros(1,L); acum_w=zeros(1,L);
erro2_sogi = zeros(1,L); sda_valfa= zeros(1,L); sda_vbeta= zeros(1,L);
vq = zeros(1,L); w = zeros(1,L); wp = zeros(1,L); wi = zeros(1,L);
sda_valfa(1)=0; sda_vbeta(1)=0; erro1_sogi(1)=0; erro2_sogi(1)=0;
wp(1)=0; wi(1)=0; w(1)=0;

pll_alfa = zeros(1,L); pll_alfa(1) = 0; pll_beta = zeros(1,L); pll_beta(1) = 0;
erro_amp = zeros(1,L); erro_amp(1) = 0; amp = zeros(1,L); amp(1) = 0;


cont = zeros(1,L);cont(1) = 0;


for k =2:L

  tempo = t.*Ts;
  cont(k) = cont(k-1) + 1;
  x1 = 5*sin(2*pi*(fm1/Fs).*t);
  x2 = 5*sin(2*pi*(fm1/Fs).*t) + (5/3)*sin(6*pi*(fm1/Fs).*t) + sin(10*pi*(fm1/Fs).*t) + (5/7)*sin(14*pi*(fm1/Fs).*t);
  if (cont(k)<20001)
    v_in(k) = x1(k);
  else
    v_in(k) = x2(k);
  endif

        %%% LOOP do SOGI %%%

        erro1_sogi(k) = ( ( K*( v_in(k)-sda_valfa(k-1) ) ) - sda_vbeta(k-1) ).*w(k-1);      
        sda_valfa(k) = sda_valfa(k-1) + (Ts/2)*(erro1_sogi(k)+erro1_sogi(k-1));
        erro2_sogi(k) = w(k-1).*sda_valfa(k-1);
        sda_vbeta(k) = sda_vbeta(k-1) + (Ts/2)*(erro2_sogi(k)+ erro2_sogi(k-1));

       
  % Malha de Fase do E-PLL modificado
 
  vq(k)= (sda_valfa(k) - pll_alfa(k-1)).*cos(teta(k-1)) + (sda_vbeta(k) - pll_beta(k-1)).*sin(teta(k-1));

        acum_w(k)= acum_w(k-1)+(vq(k)+vq(k-1))*ki*Ts/2;
        wp(k)= kp*vq(k);
        wi(k)= wo + (acum_w(k));
        w(k)=wp(k)+wi(k);

        teta(k)= teta(k-1) + (w(k)+w(k-1))*Ts/2;

    if (teta(k) > (2*pi))
           teta(k) = 0;
    endif

% Malha de Amplitude do E-PLL modificado

erro_amp(k) = (sda_valfa(k) - pll_alfa(k-1)).*sin(teta(k-1)) - (sda_vbeta(k) - pll_beta(k-1)).*cos(teta(k-1));
amp(k)= amp(k-1)+(erro_amp(k)+erro_amp(k-1))*Ka*Ts/2;

    pll_alfa =  amp(k)*sin(teta);
    pll_beta = -amp(k)*cos(teta);
    
   
endfor