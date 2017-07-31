h1 = zeros(3463,1);
h1(1:200) = mag(1:200);
h1FFT = fft(h1);
magFFT2 = magFFT;
magFFT3 = magFFT2.*h1FFT;
plot(ifft(magFFT3))