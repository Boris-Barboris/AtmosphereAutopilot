function frame = get_frame(space,buffer_size,frame_index,frame_size,fixed_size)
    fixed_frame_start = max(frame_index - fixed_size + 1, 1);
    frame = space(:,fixed_frame_start:frame_index);
    left_in_buffer = min(fixed_frame_start - 1, buffer_size - fixed_size);
    stoh_count = min(frame_size - fixed_size, left_in_buffer / 5);
    if stoh_count <= 0
        return
    end
    stoh_frame_dist = unidrnd(double(left_in_buffer - 1), 1, stoh_count);
    for i = 1:stoh_count
        frame = [space(:,fixed_frame_start - 1 - stoh_frame_dist(i)), frame];
    end
end

