  t2 = conj(q3);
  t4 = wmass*2.0;
  t8 = wmass*1.69E-2;
  t9 = q2*1.203210011390811E+2;
  t10 = q3*1.198501540420636E+2;
  t11 = q4*9.287448442614035E-1;
  t12 = q5*9.211640318625102E-1;
  t13 = dq3*2.229627428782927E+1;
  t14 = dq2*2.233333295757935E+1;
  t15 = dq5*8.472453159249492E-1;
  t16 = dq4*8.523940595548498E-1;
  t3 = cos(t2);
  t5 = t2*2.0;
  t7 = bmass+t4;
  t17 = t10+t12+t13+t15;
  t18 = t9+t11+t14+t16;
  t6 = cos(t5);
  A0[3][0] = (t18*(Ixb/2.0+Ixwr/2.0+Iywl/2.0+Izb/2.0+Izwl/2.0+Izwr/2.0+bmass*8.45E-3+t8+(Ixb*t6)/2.0+(Ixwr*t6)/2.0+(Iywl*t6)/2.0-(Izb*t6)/2.0-(Izwl*t6)/2.0-(Izwr*t6)/2.0+bmass*t6*8.45E-3+t6*t8)+gr*l*t7*sin(q2))/t3-t3*t18*(Iywl+t8);
  A0[4][0] = t17*(Ixwl+Iyb+Iywr+bmass*1.69E-2+wmass*3.38E-2)-t17*(Iywr+t8)+gr*l*t7*sin(q3);
