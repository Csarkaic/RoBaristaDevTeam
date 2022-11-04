classdef dobotstir < handle
    methods (Static)
        function dobotStir
            dobot = DobotBarista;
            
            dobotQStart = dobot.model.getpos;
            dobotPose = dobot.model.fkine(dobotQStart);
            dobotPose = dobotPose*transl(0,0,0.1);
dobotQStart = dobot.model.ikcon(dobotPose, [0,0,0,pi,0]);
dobot.model.animate(dobotQStart);

           
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
            for i = 1:1:5
                %if matlab.ui.control.StateButton==0
                    % q = dobot.model.getpos;
                    q = dobot.model.fkine(k);
                    f = q * transl(-0.02,-0.02,0);
                    l = dobot.model.ikcon(f,[k]);
                    
                    j = jtraj(k,l,steps);
                    
                    for i = 1:1:steps
                        if matlab.ui.control.StateButton==0
                            dobot.model.animate(j(i,:))
                            pause(0.01)
                        end
                   % end
                    
                    q = dobot.model.fkine(l);
                    f = q * transl(0.02,-0.02,0);
                    k = dobot.model.ikcon(f,[l]);
                    
                    j = jtraj(l,k,steps);
                    
                    for i = 1:1:steps
                       % if matlab.ui.control.StateButton==0
                            dobot.model.animate(j(i,:))
                            pause(0.01)
                       % end
                    end
                    
                    q = dobot.model.fkine(k);
                    f = q * transl(0.02,0.02,0);
                    l = dobot.model.ikcon(f,[k]);
                    
                    j = jtraj(k,l,steps);
                    
                    for i = 1:1:steps
                       % if matlab.ui.control.StateButton==0
                            dobot.model.animate(j(i,:))
                            pause(0.01)
                       % end
                    end
                    
                    q = dobot.model.fkine(l);
                    f = q * transl(-0.02,0.02,0);
                    k = dobot.model.ikcon(f,[l]);
                    
                    j = jtraj(l,k,steps);
                    
                    for i = 1:1:steps
                       % if matlab.ui.control.StateButton==0
                            dobot.model.animate(j(i,:))
                            pause(0.02)
                       % end
                    end
                end
            end
        end
    end
end

