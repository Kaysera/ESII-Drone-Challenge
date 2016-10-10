classdef KeyInput < handle
    %KeyInput - Class for obtaining keyboard input
    %   OBJ = KeyInput() creates a figure with instructions for robot
    %   keyboard control. The object keeps track of the figure and axes handles
    %   and converts keyboard input into ASCII values.
    %
    %   KeyInput methods:
    %       getKeystroke            - Returns ASCII value for keystroke
    %       closeFigure             - Closes the figure and cleans up
    %
    %   KeyInput properties:
    %       Figure                  - Stores the figure handle
    %       Axes                    - Stores the axes handle
    %
    %   
    
    properties
        Figure = [];            % Stores the figure handle
        Axes = [];              % Stores the axes handle
    end
    
    methods
        function obj = KeyInput()
            %KeyInput - Constructor for KeyInput class
            
            callstr = 'set(gcbf,''Userdata'',double(get(gcbf,''Currentcharacter''))) ; uiresume ' ;
            
            obj.Figure = figure(...
                'Name','Press a key', ...
                'KeyPressFcn',callstr, ...
                'Position',[200 100  500 300],...
                'UserData','Timeout');
            obj.Axes = axes('Color','k','Visible','Off','XLim',[0,100],'YLim',[0,100]);

            text(20,75,'W','HorizontalAlignment','center','EdgeColor','k');
            text(30,75,'E','HorizontalAlignment','center','EdgeColor','b');
            text(10,60,'A','HorizontalAlignment','center','EdgeColor','k');
            text(20,60,'S','HorizontalAlignment','center','EdgeColor','k');
            text(30,60,'D','HorizontalAlignment','center','EdgeColor','k');
            text(10,45,'Z','HorizontalAlignment','center','EdgeColor','b');
            text(20,45,'X','HorizontalAlignment','center','EdgeColor','k');
            

            
            
            text(40,75,'R','HorizontalAlignment','center','EdgeColor','r');
            text(50,60,'G','HorizontalAlignment','center','EdgeColor','r');
            text(90,75,'P','HorizontalAlignment','center','EdgeColor','r');
            text(10,75,'Q','HorizontalAlignment','center','EdgeColor','r');
            text(50,0,'Keep this figure in scope to give commands','HorizontalAlignment','center');
        end
        
        function keyout = getKeystroke(obj)
            %GETKEYSTROKE - Returns ASCII value for keystroke
            
            try
                figure(obj.Figure);
                uiwait;
                keyout = get(obj.Figure,'Userdata') ;
            catch
                keyout = 'q';
            end
        end
        
        function closeFigure(obj)
            %CLOSEFIGURE - Closes the figure and cleans up
            
            try
                figure(obj.Figure);
                close(obj.Figure);
            catch
            end
        end
    end
    
end


