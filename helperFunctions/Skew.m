function [skew] = Skew(omega)

omega_x = omega(1);
omega_y = omega(2);
omega_z = omega(3);

skew = [0 -omega_z omega_y;
        omega_z 0 -omega_x;
        -omega_y omega_x 0];

end