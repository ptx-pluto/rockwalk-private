classdef AllowedCollisionMatrix < robotics.ros.Message
    %AllowedCollisionMatrix MATLAB implementation of moveit_msgs/AllowedCollisionMatrix
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'moveit_msgs/AllowedCollisionMatrix' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'aedce13587eef0d79165a075659c1879' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        MoveitMsgsAllowedCollisionEntryClass = robotics.ros.msg.internal.MessageFactory.getClassForType('moveit_msgs/AllowedCollisionEntry') % Dispatch to MATLAB class for message type moveit_msgs/AllowedCollisionEntry
    end
    
    properties (Dependent)
        EntryNames
        EntryValues
        DefaultEntryNames
        DefaultEntryValues
    end
    
    properties (Access = protected)
        Cache = struct('EntryValues', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'DefaultEntryNames', 'DefaultEntryValues', 'EntryNames', 'EntryValues'} % List of non-constant message properties
        ROSPropertyList = {'default_entry_names', 'default_entry_values', 'entry_names', 'entry_values'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = AllowedCollisionMatrix(msg)
            %AllowedCollisionMatrix Construct the message object AllowedCollisionMatrix
            import com.mathworks.toolbox.robotics.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('robotics:ros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('robotics:ros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('robotics:ros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function entrynames = get.EntryNames(obj)
            %get.EntryNames Get the value for property EntryNames
            javaArray = obj.JavaMessage.getEntryNames;
            array = obj.readJavaArray(javaArray, 'char');
            entrynames = arrayfun(@(x) char(x), array, 'UniformOutput', false);
        end
        
        function set.EntryNames(obj, entrynames)
            %set.EntryNames Set the value for property EntryNames
            if ~isvector(entrynames) && isempty(entrynames)
                % Allow empty [] input
                entrynames = cell.empty(0,1);
            end
            
            validateattributes(entrynames, {'cell'}, {'vector'}, 'AllowedCollisionMatrix', 'EntryNames');
            if any(cellfun(@(x) ~ischar(x), entrynames))
                error(message('robotics:ros:message:CellArrayStringError', ...
                    'entrynames'));
            end
            
            javaArray = obj.JavaMessage.getEntryNames;
            array = obj.writeJavaArray(entrynames, javaArray, 'char');
            obj.JavaMessage.setEntryNames(array);
        end
        
        function entryvalues = get.EntryValues(obj)
            %get.EntryValues Get the value for property EntryValues
            if isempty(obj.Cache.EntryValues)
                javaArray = obj.JavaMessage.getEntryValues;
                array = obj.readJavaArray(javaArray, obj.MoveitMsgsAllowedCollisionEntryClass);
                obj.Cache.EntryValues = feval(obj.MoveitMsgsAllowedCollisionEntryClass, array);
            end
            entryvalues = obj.Cache.EntryValues;
        end
        
        function set.EntryValues(obj, entryvalues)
            %set.EntryValues Set the value for property EntryValues
            if ~isvector(entryvalues) && isempty(entryvalues)
                % Allow empty [] input
                entryvalues = feval([obj.MoveitMsgsAllowedCollisionEntryClass '.empty'], 0, 1);
            end
            
            validateattributes(entryvalues, {obj.MoveitMsgsAllowedCollisionEntryClass}, {'vector'}, 'AllowedCollisionMatrix', 'EntryValues');
            
            javaArray = obj.JavaMessage.getEntryValues;
            array = obj.writeJavaArray(entryvalues, javaArray, obj.MoveitMsgsAllowedCollisionEntryClass);
            obj.JavaMessage.setEntryValues(array);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.EntryValues)
                obj.Cache.EntryValues = [];
                obj.Cache.EntryValues = obj.EntryValues;
            end
        end
        
        function defaultentrynames = get.DefaultEntryNames(obj)
            %get.DefaultEntryNames Get the value for property DefaultEntryNames
            javaArray = obj.JavaMessage.getDefaultEntryNames;
            array = obj.readJavaArray(javaArray, 'char');
            defaultentrynames = arrayfun(@(x) char(x), array, 'UniformOutput', false);
        end
        
        function set.DefaultEntryNames(obj, defaultentrynames)
            %set.DefaultEntryNames Set the value for property DefaultEntryNames
            if ~isvector(defaultentrynames) && isempty(defaultentrynames)
                % Allow empty [] input
                defaultentrynames = cell.empty(0,1);
            end
            
            validateattributes(defaultentrynames, {'cell'}, {'vector'}, 'AllowedCollisionMatrix', 'DefaultEntryNames');
            if any(cellfun(@(x) ~ischar(x), defaultentrynames))
                error(message('robotics:ros:message:CellArrayStringError', ...
                    'defaultentrynames'));
            end
            
            javaArray = obj.JavaMessage.getDefaultEntryNames;
            array = obj.writeJavaArray(defaultentrynames, javaArray, 'char');
            obj.JavaMessage.setDefaultEntryNames(array);
        end
        
        function defaultentryvalues = get.DefaultEntryValues(obj)
            %get.DefaultEntryValues Get the value for property DefaultEntryValues
            javaArray = obj.JavaMessage.getDefaultEntryValues;
            array = obj.readJavaArray(javaArray, 'logical');
            defaultentryvalues = logical(array);
        end
        
        function set.DefaultEntryValues(obj, defaultentryvalues)
            %set.DefaultEntryValues Set the value for property DefaultEntryValues
            if ~isvector(defaultentryvalues) && isempty(defaultentryvalues)
                % Allow empty [] input
                defaultentryvalues = logical.empty(0,1);
            end
            
            validateattributes(defaultentryvalues, {'logical', 'numeric'}, {'vector'}, 'AllowedCollisionMatrix', 'DefaultEntryValues');
            
            javaArray = obj.JavaMessage.getDefaultEntryValues;
            array = obj.writeJavaArray(defaultentryvalues, javaArray, 'logical');
            obj.JavaMessage.setDefaultEntryValues(array);
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.EntryValues = [];
        end
        
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Clear any existing cached properties
            cpObj.resetCache;
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.EntryNames = obj.EntryNames;
            cpObj.DefaultEntryNames = obj.DefaultEntryNames;
            cpObj.DefaultEntryValues = obj.DefaultEntryValues;
            
            % Recursively copy compound properties
            cpObj.EntryValues = copy(obj.EntryValues);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.EntryNames = strObj.EntryNames;
            obj.DefaultEntryNames = strObj.DefaultEntryNames;
            obj.DefaultEntryValues = strObj.DefaultEntryValues;
            EntryValuesCell = arrayfun(@(x) feval([obj.MoveitMsgsAllowedCollisionEntryClass '.loadobj'], x), strObj.EntryValues, 'UniformOutput', false);
            obj.EntryValues = vertcat(EntryValuesCell{:});
        end
    end
    
    methods (Access = ?robotics.ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.EntryNames = obj.EntryNames;
            strObj.DefaultEntryNames = obj.DefaultEntryNames;
            strObj.DefaultEntryValues = obj.DefaultEntryValues;
            strObj.EntryValues = arrayfun(@(x) saveobj(x), obj.EntryValues);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.moveit_msgs.AllowedCollisionMatrix.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.moveit_msgs.AllowedCollisionMatrix;
            obj.reload(strObj);
        end
    end
end
