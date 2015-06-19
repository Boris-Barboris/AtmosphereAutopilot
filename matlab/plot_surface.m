function [] = plot_surface(x, y, z)
    scrsz = get(0,'ScreenSize');
    figure('Name','Surface plot',...
        'Position',[100 50 scrsz(3)*0.8 scrsz(4)*0.8])
    surf(x, y, z);
end

