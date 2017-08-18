h1 = zeros(3463,1);
h1(1:500) = mag(1:500);

n = pow2(nextpow2(3463));  % transform length
[len,~] = size(mag);


h1FFT = fft(h1);
magFFT2 = fft(mag);

% magFFT3 = magFFT2.*h1FFT;
% magFFT3 = magFFT2-h1FFT;

h1FFT_rho = abs(h1FFT);
h1FFT_theta = angle(h1FFT);

magFFT2_rho = abs(magFFT2);
magFFT2_theta = angle(magFFT2);
% magFFT2_rho = magFFT2_rho - h1FFT_rho;
% magFFT3 = magFFT2_rho .* exp(1i*magFFT2_theta);
% magFFT3 = magFFT2 .* h1FFT;
% magFFT3 = magFFT2;

meanFFT = mean(magFFT2_rho);
threshold  = 2.8*meanFFT; % Fine-tune this
lessThreshCount = 0;
for i = 1:len
    if(magFFT2_rho(i) < threshold)
        magFFT2(i) = 0;
        lessThreshCount = lessThreshCount + 1;
    end
end
lessThreshCount

magFFT2(1) = 0;
plot(ifft(magFFT2))