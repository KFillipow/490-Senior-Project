function feature_vec_write(fft_x,fft_data,group_num)
    %group = group_num + zeros(1,2048);
    relevant_amount = 1;
    relevant_x_val = zeros(1,2);
    relevant_data = zeros(1,2);
    for i = 1:2048
        if(fft_data(i) > 200000)
            relevant_data(relevant_amount) = fft_data(i);
            relevant_x_val(relevant_amount) = fft_x(i);
            relevant_amount = relevant_amount + 1;
        end
    end
    group = group_num + zeros(1,relevant_amount-1);
    %fprintf('X size %d Y size %d Group size %d',length(group),length(relevant_data),length(relevant_x_val));
    %feature_vec = [fft_x(1:2048);fft_data(1:2048);group];
    feature_vec = [relevant_x_val;relevant_data;group];
    file_id = fopen('testing_data_reduced.txt','a');
    fprintf(file_id,'%d %f %d \n',feature_vec);
end