function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
    if nargin < 6
        returnOnceFound = true;
    end
    result = false;
    
    for qIndex = 1:size(qMatrix,1)
        % Get the transform of every joint (i.e. start and end of every link)
        tr = GetLinkPoses(qMatrix(qIndex,:), robot);
    
        % Go through each link and also each triangle face
        for i = 1 : size(tr,3)-1    
            for faceIndex = 1:size(faces,1)
                vertOnPlane = vertex(faces(faceIndex,1)',:);
                [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                    result = true;
                    if returnOnceFound
                        return
                    end
                end
            end    
        end
    end

    function [ transforms ] = GetLinkPoses( q, robot)
        links = robot.links;
        transforms = zeros(4, 4, length(links) + 1);
        transforms(:,:,1) = robot.base;
        
        for i = 1:length(links)
            L = links(1,i);
            
            current_transform = transforms(:,:, i);
            
            current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
            transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
            transforms(:,:,i + 1) = current_transform;
        end
    end

