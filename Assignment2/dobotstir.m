close all; 
clear all; 
hold on;

dobot = DobotBarista; 
%%
steps = 5; 
g = dobot.model.getpos;
q = dobot.model.fkine(g);
f = q * transl(0,0.02,0);
k = dobot.model.ikcon(f,[g]);

j = jtraj(g,k,steps);

for i = 1:1:steps
    dobot.model.animate(j(i,:))
    pause(0.01)
end

% q = dobot.model.getpos; 
q = dobot.model.fkine(k);
f = q * transl(-0.02,-0.02,0);
l = dobot.model.ikcon(f,[k]);

j = jtraj(k,l,steps);

for i = 1:1:steps
    dobot.model.animate(j(i,:))
    pause(0.01)
end

q = dobot.model.fkine(l);
f = q * transl(0.02,-0.02,0);
k = dobot.model.ikcon(f,[l]);

j = jtraj(l,k,steps);

for i = 1:1:steps
    dobot.model.animate(j(i,:))
    pause(0.01)
end

q = dobot.model.fkine(k);
f = q * transl(0.02,0.02,0);
l = dobot.model.ikcon(f,[k]);

j = jtraj(k,l,steps);

for i = 1:1:steps
    dobot.model.animate(j(i,:))
    pause(0.01)
end

q = dobot.model.fkine(l);
f = q * transl(-0.02,0.02,0);
k = dobot.model.ikcon(f,[l]);

j = jtraj(l,k,steps);

for i = 1:1:steps
    dobot.model.animate(j(i,:))
    pause(0.02)
end


