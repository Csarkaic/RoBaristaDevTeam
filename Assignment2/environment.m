classdef environment < handle
    properties
%         Kukbot = Kuka;
%         dobot = DobotBarista;
    end
    
    methods (Static)
        function spawnEnvironment
<<<<<<< HEAD
            hold on;
=======
            hold on;            
>>>>>>> facbf0153e541476e6619ad521f2ee3ef4769e01
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

            for i = 0.6:0.266:1.4
                for j = 0.65:0.2:1.25
                    [f,v,data] = plyread('croissant.ply', 'tri');
                    %Set the color variables for object file
                    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
                    %set the location into the workspace using [x,y,z coordinates]
                    location_fencing = [i,-0.5,j];
                    %load the table through trisurf(triangular surface) reading and location
                    fencing = trisurf(f, v(:,1)+location_fencing(1,1),v(:,2)+location_fencing(1,2),...
                        v(:,3)+location_fencing(1,3),'FaceVertexCData',vertexColours,'EdgeColor','interp',...
                        'EdgeLighting','flat');
                end
            end
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
        
    end
    
end