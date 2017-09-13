function waypoints = extract_waypoints(filename)
% Workspace struct must be called 'richie_cursive'... for now
   load(filename);
   waypoints = zeros(length(richie_cursive),2);
   for i = 1 : length(richie_cursive)
       waypoints(length(richie_cursive)-(i-1),1:2) = richie_cursive(i).Position;
   end
end