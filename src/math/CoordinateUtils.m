classdef CoordinateUtils
    %COORDINATEUTILS 2D frame transformations and radar geometry.
    %   SO(2) rotations, inertial/body/radar frames.
    %   Radar is side-looking: boresight = inward normal to circular path.
    
    methods (Static)
        function R = rotationSO2(angle_rad)
            %ROTATIONSO2 SO(2) rotation matrix for angle (rad).
            %   R rotates from body to world: p_world = R * p_body.
            %   Body x = [cos(angle); sin(angle)], body y = [-sin(angle); cos(angle)].
            c = cos(angle_rad);
            s = sin(angle_rad);
            R = [c -s; s c];
        end
        
        function p_body = worldToBody(p_world, theta_rad)
            %WORLDTOBODY Transform vector from inertial to aircraft body frame.
            %   p_body = R_a' * p_world, with R_a = rotationSO2(theta_rad).
            R = CoordinateUtils.rotationSO2(theta_rad);
            p_body = R' * p_world;
        end
        
        function n_in = inwardNormal(p_aircraft, center)
            %INWARDNORMAL Unit inward normal (toward circle center).
            %   n_in = (center - p_aircraft) / norm(...).
            delta = center(:) - p_aircraft(:);
            n = norm(delta);
            if n < 1e-10
                error('CoordinateUtils:Degenerate', 'Aircraft at center.');
            end
            n_in = delta / n;
        end
        
        function t = tangentVector(p_aircraft, center)
            %TANGENTVECTOR Unit tangent (velocity direction for counter-clockwise).
            %   t perpendicular to (p_aircraft - center), pointing forward along orbit.
            delta = p_aircraft(:) - center(:);
            R = norm(delta);
            if R < 1e-10
                error('CoordinateUtils:Degenerate', 'Aircraft at center.');
            end
            % Rotate radius by 90°: (x,y) -> (-y,x) gives tangent for CCW.
            t = [-delta(2); delta(1)] / R;
        end
        
        function p_radar = worldToRadarFrame(p_rel_world, p_aircraft, center)
            %WORLDTORADARFRAME Transform relative position to radar frame.
            %   p_rel_world = p_target - p_aircraft (inertial).
            %   Radar x-axis = inward normal, y-axis = tangent.
            %   Returns p_radar = [x_r; y_r] with rho = norm(p_radar), phi = atan2(y_r, x_r).
            n_in = CoordinateUtils.inwardNormal(p_aircraft, center);
            t = CoordinateUtils.tangentVector(p_aircraft, center);
            R_radar_cols = [n_in, t]; % columns = radar axes in world
            p_radar = R_radar_cols' * p_rel_world(:);
        end
    end
end
