
classdef helper < handle

    properties(Access = public)
        posArray = [];
    end

    methods

        % function self = helper()
        %     self.posArray = [];
        % end
        function objectArr = placeXandO(self, filename, objectArr, posArm)
            for z = 0:0.05:0.1
                for i = -0.25:-0.1:-0.35
                    for y = -0.25:0.1:0.15
                    objectArr = [PlaceObject(filename, [posArm(1, 4) + y, posArm(2, 4) - i, z]);objectArr];
                 
                    self.posArray = [([posArm(1, 4) + y, posArm(2, 4) - i, z]); self.posArray;];
                    end
                end
            end
        end

        function res = objLocation(self, iter)
            res = self.posArray(iter, :);
        end
    end
end