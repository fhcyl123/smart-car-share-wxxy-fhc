	function dy=dym(t,y,flag,uk)

	c0=260; 
	c1=2.5; 
	c2=0.02;
	Fc=0.28; 
	Fs=0.34;
	Vs=0.01;
	J=1.0;

	dy=[0,0,0]';

	g=Fc+(Fs-Fc)*exp(-(y(2)/Vs)^2)+c2*y(2);

	dy(3)=y(2)-(c0*abs(y(2))/g)*y(3);
	F=c0*y(3)+c1*dy(3)+c2*y(2);
	dy(2)=1/J*(uk-F);
	dy(1)=y(2);