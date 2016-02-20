% Part 1 predicted
lv = 0.25;
la = 0;
t = 4;
dt = .1;

predictor(t,dt,la,lv,'1');

lv = 0.25;
la = 1;
t = 4;
dt = .1;
    
predictor(t,dt,la,lv,'2');   

lv = 0.25;
la = -1;
t = 4;
dt = .1;

predictor(t,dt,la,lv,'3');

lv = 0;
la = 0;
t = 10;
dt = .1;

predictor2(t,dt,la,lv,'4');

lv = 0;
la = 0;
t = 10;
dt = .1;

predictor2(t,dt,la,lv,'5');