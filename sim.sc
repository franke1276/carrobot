Ks=0.72;                            // Verstärkung der Strecke
T1=0.11;                            // Zeitkonstante 1
T2=0.005;                           // Zeitkonstante 2
Kp=18;                              // Proportionalbeiwert
Ki=170;                             // Integralbeiwert
Kd=0;                               // Differenzialbeiwert
 
s=poly(0,'s');                      // definiert s als Polynomvariable
P=(T1*s+1)*(T2*s+1);                // Streckencharakteristik
Gs=syslin('c',Ks,P)                 // Übertragungsfunktion der Strecke

RZ=poly([Ki Kp Kd],'s','coeff')     // Zählerpolynom des Reglers
RN=poly([0 1],'s','coeff')          // Nennerpolynom des Reglers
Gr=syslin('c',RZ,RN)                // Übertragungsfunktion des Reglers

G=Gr*Gs                             // Übertragungsfunktion gesamt
xset("window",0);
xbasc(0);
bode([Gr;G;Gs],0.1,100,['Regler';'gesamt';'Strecke'])

Gcl=G/(G+1)                         // geschlossene Regelschleife
t=0:0.001:0.2;
y=csim('step',t,Gcl);               // berechnet Sprungantwort
xset("window",1);

plot2d(t,y,2);
xgrid();
xtitle("Sprungantwort","sec");