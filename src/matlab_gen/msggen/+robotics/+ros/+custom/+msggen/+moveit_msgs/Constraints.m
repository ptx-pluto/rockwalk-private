classdef Constraints < robotics.ros.Message
    %Constraints MATLAB implementation of moveit_msgs/Constraints
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'moveit_msgs/Constraints' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '8d5ce8d34ef26c65fb5d43c9e99bf6e0' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        MoveitMsgsJointConstraintClass = robotics.ros.msg.internal.MessageFactory.getClassForType('moveit_msgs/JointConstraint') % Dispatch to MATLAB class for message type moveit_msgs/JointConstraint
        MoveitMsgsOrientationConstraintClass = robotics.ros.msg.internal.MessageFactory.getClassForType('moveit_msgs/OrientationConstraint') % Dispatch to MATLAB class for message type moveit_msgs/OrientationConstraint
        MoveitMsgsPositionConstraintClass = robotics.ros.msg.internal.MessageFactory.getClassForType('moveit_msgs/PositionConstraint') % Dispatch to MATLAB class for message type moveit_msgs/PositionConstraint
        MoveitMsgsVisibilityConstraintClass = robotics.ros.msg.internal.MessageFactory.getClassForType('moveit_msgs/VisibilityConstraint') % Dispatch to MATLAB class for message type moveit_msgs/VisibilityConstraint
    end
    
    properties (Dependent)
        Name
        JointConstraints
        PositionConstraints
        OrientationConstraints
        VisibilityConstraints
    end
    
    properties (Access = protected)
        Cache = struct('JointConstraints', [], 'PositionConstraints', [], 'OrientationConstraints', [], 'VisibilityConstraints', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'JointConstraints', 'Name', 'OrientationConstraints', 'PositionConstraints', 'VisibilityConstraints'} % List of non-constant message properties
        ROSPropertyList = {'joint_constraints', 'name', 'orientation_constraints', 'position_constraints', 'visibility_constraints'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = Constraints(msg)
            %Constraints Construct the message object Constraints
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
        
        function name = get.Name(obj)
            %get.Name Get the value for property Name
            name = char(obj.JavaMessage.getName);
        end
        
        function set.Name(obj, name)
            %set.Name Set the value for property Name
            validateattributes(name, {'char'}, {}, 'Constraints', 'Name');
            
            obj.JavaMessage.setName(name);
        end
        
        function jointconstraints = get.JointConstraints(obj)
            %get.JointConstraints Get the value for property JointConstraints
            if isempty(obj.Cache.JointConstraints)
                javaArray = obj.JavaMessage.getJointConstraints;
                array = obj.readJavaArray(javaArray, obj.MoveitMsgsJointConstraintClass);
                obj.Cache.JointConstraints = feval(obj.MoveitMsgsJointConstraintClass, array);
            end
            jointconstraints = obj.Cache.JointConstraints;
        end
        
        function set.JointConstraints(obj, jointconstraints)
            %set.JointConstraints Set the value for property JointConstraints
            if ~isvector(jointconstraints) && isempty(jointconstraints)
                % Allow empty [] input
                jointconstraints = feval([obj.MoveitMsgsJointConstraintClass '.empty'], 0, 1);
            end
            
            validateattributes(jointconstraints, {obj.MoveitMsgsJointConstraintClass}, {'vector'}, 'Constraints', 'JointConstraints');
            
            javaArray = obj.JavaMessage.getJointConstraints;
            array = obj.writeJavaArray(jointconstraints, javaArray, obj.MoveitMsgsJointConstraintClass);
            obj.JavaMessage.setJointConstraints(array);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.JointConstraints)
                obj.Cache.JointConstraints = [];
                obj.Cache.JointConstraints = obj.JointConstraints;
            end
        end
        
        function positionconstraints = get.PositionConstraints(obj)
            %get.PositionConstraints Get the value for property PositionConstraints
            if isempty(obj.Cache.PositionConstraints)
                javaArray = obj.JavaMessage.getPositionConstraints;
                array = obj.readJavaArray(javaArray, obj.MoveitMsgsPositionConstraintClass);
                obj.Cache.PositionConstraints = feval(obj.MoveitMsgsPositionConstraintClass, array);
            end
            positionconstraints = obj.Cache.PositionConstraints;
        end
        
        function set.PositionConstraints(obj, positionconstraints)
            %set.PositionConstraints Set the value for property PositionConstraints
            if ~isvector(positionconstraints) && isempty(positionconstraints)
                % Allow empty [] input
                positionconstraints = feval([obj.MoveitMsgsPositionConstraintClass '.empty'], 0, 1);
            end
            
            validateattributes(positionconstraints, {obj.MoveitMsgsPositionConstraintClass}, {'vector'}, 'Constraints', 'PositionConstraints');
            
            javaArray = obj.JavaMessage.getPositionConstraints;
            array = obj.writeJavaArray(positionconstraints, javaArray, obj.MoveitMsgsPositionConstraintClass);
            obj.JavaMessage.setPositionConstraints(array);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.PositionConstraints)
                obj.Cache.PositionConstraints = [];
                obj.Cache.PositionConstraints = obj.PositionConstraints;
            end
        end
        
        function orientationconstraints = get.OrientationConstraints(obj)
            %get.OrientationConstraints Get the value for property OrientationConstraints
            if isempty(obj.Cache.OrientationConstraints)
                javaArray = obj.JavaMessage.getOrientationConstraints;
                array = obj.readJavaArray(javaArray, obj.MoveitMsgsOrientationConstraintClass);
                obj.Cache.OrientationConstraints = feval(obj.MoveitMsgsOrientationConstraintClass, array);
            end
            orientationconstraints = obj.Cache.OrientationConstraints;
        end
        
        function set.OrientationConstraints(obj, orientationconstraints)
            %set.OrientationConstraints Set the value for property OrientationConstraints
            if ~isvector(orientationconstraints) && isempty(orientationconstraints)
                % Allow empty [] input
                orientationconstraints = feval([obj.MoveitMsgsOrientationConstraintClass '.empty'], 0, 1);
            end
            
            validateattributes(orientationconstraints, {obj.MoveitMsgsOrientationConstraintClass}, {'vector'}, 'Constraints', 'OrientationConstraints');
            
            javaArray = obj.JavaMessage.getOrientationConstraints;
            array = obj.writeJavaArray(orientationconstraints, javaArray, obj.MoveitMsgsOrientationConstraintClass);
            obj.JavaMessage.setOrientationConstraints(array);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.OrientationConstraints)
                obj.Cache.OrientationConstraints = [];
                obj.Cache.OrientationConstraints = obj.OrientationConstraints;
            end
        end
        
        function visibilityconstraints = get.VisibilityConstraints(obj)
            %get.VisibilityConstraints Get the value for property VisibilityConstraints
            if isempty(obj.Cache.VisibilityConstraints)
                javaArray = obj.JavaMessage.getVisibilityConstraints;
                array = obj.readJavaArray(javaArray, obj.MoveitMsgsVisibilityConstraintClass);
                obj.Cache.VisibilityConstraints = feval(obj.MoveitMsgsVisibilityConstraintClass, array);
            end
            visibilityconstraints = obj.Cache.VisibilityConstraints;
        end
        
        function set.VisibilityConstraints(obj, visibilityconstraints)
            %set.VisibilityConstraints Set the value for property VisibilityConstraints
            if ~isvector(visibilityconstraints) && isempty(visibilityconstraints)
                % Allow empty [] input
                visibilityconstraints = feval([obj.MoveitMsgsVisibilityConstraintClass '.empty'], 0, 1);
            end
            
            validateattributes(visibilityconstraints, {obj.MoveitMsgsVisibilityConstraintClass}, {'vector'}, 'Constraints', 'VisibilityConstraints');
            
            javaArray = obj.JavaMessage.getVisibilityConstraints;
            array = obj.writeJavaArray(visibilityconstraints, javaArray, obj.MoveitMsgsVisibilityConstraintClass);
            obj.JavaMessage.setVisibilityConstraints(array);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.VisibilityConstraints)
                obj.Cache.VisibilityConstraints = [];
                obj.Cache.VisibilityConstraints = obj.VisibilityConstraints;
            end
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.JointConstraints = [];
            obj.Cache.PositionConstraints = [];
            obj.Cache.OrientationConstraints = [];
            obj.Cache.VisibilityConstraints = [];
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
            cpObj.Name = obj.Name;
            
            % Recursively copy compound properties
            cpObj.JointConstraints = copy(obj.JointConstraints);
            cpObj.PositionConstraints = copy(obj.PositionConstraints);
            cpObj.OrientationConstraints = copy(obj.OrientationConstraints);
            cpObj.VisibilityConstraints = copy(obj.VisibilityConstraints);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Name = strObj.Name;
            JointConstraintsCell = arrayfun(@(x) feval([obj.MoveitMsgsJointConstraintClass '.loadobj'], x), strObj.JointConstraints, 'UniformOutput', false);
            obj.JointConstraints = vertcat(JointConstraintsCell{:});
            PositionConstraintsCell = arrayfun(@(x) feval([obj.MoveitMsgsPositionConstraintClass '.loadobj'], x), strObj.PositionConstraints, 'UniformOutput', false);
            obj.PositionConstraints = vertcat(PositionConstraintsCell{:});
            OrientationConstraintsCell = arrayfun(@(x) feval([obj.MoveitMsgsOrientationConstraintClass '.loadobj'], x), strObj.OrientationConstraints, 'UniformOutput', false);
            obj.OrientationConstraints = vertcat(OrientationConstraintsCell{:});
            VisibilityConstraintsCell = arrayfun(@(x) feval([obj.MoveitMsgsVisibilityConstraintClass '.loadobj'], x), strObj.VisibilityConstraints, 'UniformOutput', false);
            obj.VisibilityConstraints = vertcat(VisibilityConstraintsCell{:});
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
            
            strObj.Name = obj.Name;
            strObj.JointConstraints = arrayfun(@(x) saveobj(x), obj.JointConstraints);
            strObj.PositionConstraints = arrayfun(@(x) saveobj(x), obj.PositionConstraints);
            strObj.OrientationConstraints = arrayfun(@(x) saveobj(x), obj.OrientationConstraints);
            strObj.VisibilityConstraints = arrayfun(@(x) saveobj(x), obj.VisibilityConstraints);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.moveit_msgs.Constraints.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.moveit_msgs.Constraints;
            obj.reload(strObj);
        end
    end
end
