index = 300;
[x_mesh, y_mesh] = meshgrid(linspace(param_values(1,index)-500,param_values(1,index)+15,100),...
    linspace(-100,500,100));
frame = get_frame(state_space,300,index,300,300);
z_mesh = arrayfun(@(x,y)error_function([x param_values(1,index) y param_values(4,index)],...
    moi, frame(1,:), frame(2,:), frame(3,:), frame(4,:)),x_mesh,y_mesh);
plot_surface(x_mesh, y_mesh, z_mesh)
xlabel('k aoa')
ylabel('k input');