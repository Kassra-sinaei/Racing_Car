classdef Simulation
    %SIMULATION of Race Car with Non-linear tire model
    % Refer to Learning How to Autonomously Race a Car: A Predictive
    % Control Approach (2020) for more detai.

    
    properties
        m_          % mass
        i_          % inertial moment
        lr_         % wheel axle distance from CoG
        lf_         % wheel axle distance from CoG

        ptv_        % torque vectoring gain

        df_
        cf_
        bf_
        dr_
        cr_
        br_

        X_          % global X
        Y_          % global Y
        psi_        % Heading

        dt_
        deltaT_

        x_

        track_
    end
    
    methods
        function obj = Simulation(track, m, I, l_r, l_f, p_tv, deltaT, dt)
            %SIMULATION Construct an instance of this class
            %   Initialize system's property
            obj.m_ = m; obj.i_ = I; obj.lf_ = l_f; obj.lr_ = l_r; 
            obj.ptv_ = p_tv;
            obj.df_ = 0.8 * obj.m_ * 9.81 / 2.0;
            obj.dr_ = 0.8 * obj.m_ * 9.81 / 2.0;
            obj.bf_ = 1.0; obj.br_ = 1.0;
            obj.cf_ = 1.25; obj.cr_ = 1.25;
            
            obj.track_ = track;

            obj.x_ = zeros(6, 1);
            obj.deltaT_ = deltaT; obj.dt_ = dt;
            obj.X_ = obj.track_(1, 1); obj.Y_ = obj.track_(1, 2);
            obj.psi_ = 0.5 * (obj.track_(end, 3) + obj.track_(1, 3));

        end

        function obj = step(obj, u)
            iter = 1;
            while((iter) * obj.deltaT_ <= obj.dt_)
                % Compute tire forces
                alpha_f = u(1) - atan2( obj.x_(2) + obj.lf_ * obj.x_(3), obj.x_(1) );
                alpha_r = - atan2( obj.x_(2) - obj.lf_ * obj.x_(3) , obj.x_(1));
                Fyf = obj.df_ * sin( obj.cf_ * atan(obj.bf_ * alpha_f ) );
                Fyr = obj.dr_ * sin( obj.cr_ * atan(obj.br_ * alpha_r ) );
                
                cur = obj.curvature(obj.x_(5));
                obj.x_ = obj.x_ + ...
                    obj.deltaT_ * [u(2) - 1 / obj.m_ * Fyf * sin(u(1)) + obj.x_(3) * obj.x_(2);
                                1 / obj.m_ * (Fyf * cos(u(1)) + Fyr) - obj.x_(3) * obj.x_(1);
                                1 / obj.i_ *(obj.lf_ * Fyf * cos(u(1)) - obj.lr_ * Fyr);
                                obj.x_(3) - (obj.x_(1) * cos(obj.x_(4)) - obj.x_(2) * sin(obj.x_(4))) / (1 - cur * obj.x_(6)) * cur;
                                (obj.x_(1) * cos(obj.x_(4)) - obj.x_(2) * sin(obj.x_(4))) / (1 - cur * obj.x_(6));
                                obj.x_(1) * sin(obj.x_(4)) + obj.x_(2) * cos(obj.x_(4))];
                obj.psi_ = obj.psi_ + obj.deltaT_ * obj.x_(3);
                obj.X_ = obj.X_ + obj.deltaT_ * (obj.x_(1) * cos(obj.psi_) - obj.x_(2) * sin(obj.psi_));
                obj.Y_ = obj.Y_ + obj.deltaT_ * (obj.x_(1) * sin(obj.psi_) + obj.x_(2) * cos(obj.psi_));
                iter = iter + 1;
            end
        end

        function res = curvature(obj, s)
            i = 1;
            while obj.track_(i, 6) < s
                i = i+1;
            end
            if i == 1
                res = 0.5 * (obj.track_(end, 4) + obj.track_(i, 4));
            else
                res = 0.5 * (obj.track_(i-1, 4) + obj.track_(i, 4));
            end
        end
    end
end

