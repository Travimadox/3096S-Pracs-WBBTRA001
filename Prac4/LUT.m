% Number of points
N = 128;

% Initialize vectors for LUTs
sinusoid_LUT = zeros(1, N);
sawtooth_LUT = zeros(1, N);
triangular_LUT = zeros(1, N);

% Create LUT for sinusoid
for i = 1:N
    sinusoid_LUT(i) = round((sin(2 * pi * (i - 1) / N) + 1) * 511.5); % scaling and shifting to fit into 0-1023 range
end

% Create LUT for sawtooth wave
for i = 1:N
    sawtooth_LUT(i) = round((i - 1) / (N - 1) * 1023); % scaling to fit into 0-1023 range
end

% Create LUT for triangular wave
for i = 1:N
    if i <= N/2
        triangular_LUT(i) = round((i - 1) / (N/2 - 1) * 1023); % first half of the wave
    else
        triangular_LUT(i) = round((N - i) / (N/2 - 1) * 1023); % second half of the wave
    end
end

% Plot LUTs for verification
figure;

subplot(3, 1, 1);
plot(sinusoid_LUT);
title('Sinusoid LUT');
xlabel('Index');
ylabel('Value');

subplot(3, 1, 2);
plot(sawtooth_LUT);
title('Sawtooth LUT');
xlabel('Index');
ylabel('Value');

subplot(3, 1, 3);
plot(triangular_LUT);
title('Triangular LUT');
xlabel('Index');
ylabel('Value');

% Copy LUTs to a C code as arrays
fprintf('Copy these arrays into your main.c:\n');

fprintf('int sinusoid_LUT[128] = {');
fprintf('%d, ', sinusoid_LUT(1:end-1));
fprintf('%d};\n', sinusoid_LUT(end));

fprintf('int sawtooth_LUT[128] = {');
fprintf('%d, ', sawtooth_LUT(1:end-1));
fprintf('%d};\n', sawtooth_LUT(end));

fprintf('int triangular_LUT[128] = {');
fprintf('%d, ', triangular_LUT(1:end-1));
fprintf('%d};\n', triangular_LUT(end));
