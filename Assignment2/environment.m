classdef environment < handle
    properties
%         Kukbot = Kuka;
%         dobot = DobotBarista;
    end
    
    methods (Static)
        function spawnEnvironment
            hold on;

            Kukbot = Kuka;
            dobot = DobotBarista;
            
            %             q = dobot.model.getpos;
            %             f = dobot.model.fkine(q);
            %             f = f * transl(0,0,0.3);
            %             i = dobot.model.ikcon(f,[q]);
            %             dobot.model.animate(i);
            %
            % dobot.model.animate(dobot.model.base);
            
            
            surf([-2.5,-2.5;2.5,2.5],[-1,2.5;-1,2.5],[0,0;0,0]...
                ,'CData',imread('floor1.jpg'),'FaceColor','texturemap');
            
            surf([-2.5,2.5;-2.5,2.5],[-1,-1;-1,-1],[0,0;1.5,1.5]...
                ,'CData',imread('wall.jpg'),'FaceColor','texturemap');
            
            
            [f,v,data] = plyread('FINALcounter.ply', 'tri');
            %Set the color variables for object file
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            %set the location into the workspace using [x,y,z coordinates]
            location_FINALcounter = [-0,-0.99,0];
            %load the table through trisurf(triangular surface) reading and location
            FINALcounter = trisurf(f, v(:,1)+location_FINALcounter(1,1),v(:,2)+location_FINALcounter(1,2),...
                v(:,3)+location_FINALcounter(1,3),'FaceVertexCData',vertexColours,'EdgeColor','interp',...
                'EdgeLighting','flat');
            
            [f,v,data] = plyread('FINALtable.ply', 'tri');
            %Set the color variables for object file
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            %set the location into the workspace using [x,y,z coordinates]
            location_FINALtable = [0,0.8,0];
            %load the table through trisurf(triangular surface) reading and location
            FINALtable = trisurf(f, v(:,1)+location_FINALtable(1,1),v(:,2)+location_FINALtable(1,2),...
                v(:,3)+location_FINALtable(1,3),'FaceVertexCData',vertexColours,'EdgeColor','interp',...
                'EdgeLighting','flat');
            
            [f,v,data] = plyread('FINALcoffeemachine.ply', 'tri');
            %Set the color variables for object file
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            %set the location into the workspace using [x,y,z coordinates]
            location_FINALcoffeemachine = [-0.7,-0.8,0.55];
            %load the table through trisurf(triangular surface) reading and location
            FINALcoffeemachine = trisurf(f, v(:,1)+location_FINALcoffeemachine(1,1),v(:,2)+location_FINALcoffeemachine(1,2),...
                v(:,3)+location_FINALcoffeemachine(1,3),'FaceVertexCData',vertexColours,'EdgeColor','interp',...
                'EdgeLighting','flat');
            
            [f,v,data] = plyread('FINALgrinder.ply', 'tri');
            %Set the color variables for object file
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            %set the location into the workspace using [x,y,z coordinates]
            location_FINALgrinder = [-0.9,1.2,0.55];
            %load the table through trisurf(triangular surface) reading and location
            FINALgrinder = trisurf(f, v(:,1)+location_FINALgrinder(1,1),v(:,2)+location_FINALgrinder(1,2),...
                v(:,3)+location_FINALgrinder(1,3),'FaceVertexCData',vertexColours,'EdgeColor','interp',...
                'EdgeLighting','flat');
            
            [f,v,data] = plyread('FINALtamper.ply', 'tri');
            %Set the color variables for object file
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            %set the location into the workspace using [x,y,z coordinates]
            location_FINALtamper = [-0.65,1.2,0.55];
            %load the table through trisurf(triangular surface) reading and location
            FINALtamper = trisurf(f, v(:,1)+location_FINALtamper(1,1),v(:,2)+location_FINALtamper(1,2),...
                v(:,3)+location_FINALtamper(1,3),'FaceVertexCData',vertexColours,'EdgeColor','interp',...
                'EdgeLighting','flat');
            
            [f,v,data] = plyread('FINALfridge.ply', 'tri');
            %Set the color variables for object file
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            %set the location into the workspace using [x,y,z coordinates]
            location_FINALtamper = [0.75,0.9,0.55];
            %load the table through trisurf(triangular surface) reading and location
            FINALfridge = trisurf(f, v(:,1)+location_FINALtamper(1,1),v(:,2)+location_FINALtamper(1,2),...
                v(:,3)+location_FINALtamper(1,3),'FaceVertexCData',vertexColours,'EdgeColor','interp',...
                'EdgeLighting','flat');
            
            [f,v,data] = plyread('FINALshelf.ply', 'tri');
            %Set the color variables for object file
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            %set the location into the workspace using [x,y,z coordinates]
            location_FINALshelf = [1,-0.8,0.55];
            %load the table through trisurf(triangular surface) reading and location
            FINALshelf = trisurf(f, v(:,1)+location_FINALshelf(1,1),v(:,2)+location_FINALshelf(1,2),...
                v(:,3)+location_FINALshelf(1,3),'FaceVertexCData',vertexColours,'EdgeColor','interp',...
                'EdgeLighting','flat');
            
            [f,v,data] = plyread('fireextinguisher.ply', 'tri');
            %Set the color variables for object file
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            %set the location into the workspace using [x,y,z coordinates]
            location_fireextinguisher = [-2.3,-0.95,0];
            %load the table through trisurf(triangular surface) reading and location
            fireextinguisher = trisurf(f, v(:,1)+location_fireextinguisher(1,1),v(:,2)+location_fireextinguisher(1,2),...
                v(:,3)+location_fireextinguisher(1,3),'FaceVertexCData',vertexColours,'EdgeColor','interp',...
                'EdgeLighting','flat');

            [f,v,data] = plyread('estop.ply', 'tri');
            %Set the color variables for object file
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            %set the location into the workspace using [x,y,z coordinates]
            location_estop = [-2.3,-0.99,0.75];
            %load the table through trisurf(triangular surface) reading and location
            estop = trisurf(f, v(:,1)+location_estop(1,1),v(:,2)+location_estop(1,2),...
                v(:,3)+location_estop(1,3),'FaceVertexCData',vertexColours,'EdgeColor','interp',...
                'EdgeLighting','flat');
            
            [f,v,data] = plyread('lightcurtain_back.ply', 'tri');
            %Set the color variables for object file
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            %set the location into the workspace using [x,y,z coordinates]
            location_lightcurtain_back = [-2,-0.99,0];
            %load the table through trisurf(triangular surface) reading and location
            lightcurtain_back = trisurf(f, v(:,1)+location_lightcurtain_back(1,1),v(:,2)+location_lightcurtain_back(1,2),...
                v(:,3)+location_lightcurtain_back(1,3),'FaceVertexCData',vertexColours,'EdgeColor','interp',...
                'EdgeLighting','flat');

            [f,v,data] = plyread('lightcurtain_front.ply', 'tri');
            %Set the color variables for object file
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            %set the location into the workspace using [x,y,z coordinates]
            location_lightcurtain_front = [-2,1,0];
            %load the table through trisurf(triangular surface) reading and location
            lightcurtain_front = trisurf(f, v(:,1)+location_lightcurtain_front(1,1),v(:,2)+location_lightcurtain_front(1,2),...
                v(:,3)+location_lightcurtain_front(1,3),'FaceVertexCData',vertexColours,'EdgeColor','interp',...
                'EdgeLighting','flat');

            [f,v,data] = plyread('fencing.ply', 'tri');
            %Set the color variables for object file
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            %set the location into the workspace using [x,y,z coordinates]
            location_fencing = [-1.6,1.2,0];
            %load the table through trisurf(triangular surface) reading and location
            fencing = trisurf(f, v(:,1)+location_fencing(1,1),v(:,2)+location_fencing(1,2),...
                v(:,3)+location_fencing(1,3),'FaceVertexCData',vertexColours,'EdgeColor','interp',...
                'EdgeLighting','flat');
        end
        
        
    end
    
    methods (Static)
        function DobotSlider(n, value)
            q = value;
            dobot = DobotBarista;
            currentQ = dobot.model.getpos();
            currentQ(1,n) = deg2rad(q);
            dobot.model.animate(currentQ);
            drawnow();
        end
        
         function KukaSlider(n, value)
            q = value;
            kukBot = Kuka;
            currentQ = kukBot.model.getpos();
            currentQ(1,n) = deg2rad(q);
            kukBot.model.animate(currentQ);
            drawnow();
         end
         
         function doBotEndEffector(x, y, z)
             dobot = DobotBarista;
             % create transform
             T = transl(x,y,z);
             % get starting q value of Dobot
             startQ = dobot.model.getpos();
             % using trapezoidal trajectory
             steps = 50;
             endQ = dobot.model.ikcon(T,[0 pi/4 pi/2 pi/4 0]);
             qMatrix = zeros(steps,5);
             trajectory = lspb(0,1,steps);
             for i = 1:steps
                 qMatrix(i,:) = (1-trajectory(i))*startQ + trajectory(i)*endQ;
             end
             
             for i = 1:steps
                 newQ = dobot.model.getpos();
                 dobot.model.animate(newQ(i,:));
                 drawnow();                 
             end
         end
         
         function kukaEndEffector(x, y, z)
             kukBot = Kuka;
             % create transform
             T = transl(x,y,z);
             % get starting q value of Kuka Robot
             startQ = kukBot.model.getpos();
             % using trapezoidal trajectory
             steps = 50;
             endQ = kukBot.model.ikcon(T,[0 pi/4 pi/2 pi/4 pi/2 pi/2 0]);
             qMatrix = zeros(steps,7);
             trajectory = lspb(0,1,steps);
             for i = 1:steps
                 qMatrix(i,:) = (1-trajectory(i))*startQ + trajectory(i)*endQ;
             end
             
             for i = 1:steps
                 newQ = kukBot.model.getpos();
                 q2 = newQ(2);
                 q3 = newQ(3);
                 q4 = newQ(4);
                 q5 = newQ(5);
                 q6 = pi/2 - q2 - q3 - q4 - q5;
                 qMatrix(i,6) = q6;
                 kukBot.model.animate(qMatrix(i,:));
                 drawnow();
             end
             
         end
        
    end
    
end