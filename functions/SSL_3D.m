function DOAs = SSL_3D(filename,angularSpectrumMeth)

    [x,fs] = audioread(filename); % Read audio file

    %% Microphone array parameters
    % Cartesian coordinates of the microphones (in meters)
    % Warning: All microphones are supposed to be omnidirectional!
    micPos = ... 
    ...%  mic1	  mic2      mic3       mic4       mic5      mic6   
    [     0.1200   -0.0600   -0.0600   -0.0606    0.0606    0.0000;
          0         0.1039   -0.1039   -0.0350   -0.0350    0.0700;
          0.1040    0.1040    0.1040    0.2080    0.2080    0.2080]; % z

    isArrayMoving   = 0;  % The microphone array is not moving
    subArray        = []; % []: all microphones are used
    sceneTimeStamps = []; % Both array and sources are statics => no time stamps

    %% MBSS Locate core Parameters
    % localization method
    % angularSpectrumMeth      = 'GCC-PHAT'; % Local angular spectrum method {'GCC-PHAT' 'GCC-NONLIN' 'MVDR' 'MVDRW' 'DS' 'DSW' 'DNM'}
    pooling                    = 'max';      % Pooling method {'max' 'sum'}
    applySpecInstNormalization = 0;          % 1: Normalize instantaneous local angular spectra - 0: No normalization
    % Search space
    azBound                    = [-180 180]; % Azimuth search boundaries (°)
    elBound                    = [-90 90];   % Elevation search boundaries (°)
    gridRes                    = 1;          % Resolution (°) of the global 3D reference system {azimuth,elevation}
    alphaRes                   = 5;          % Resolution (°) of the 2D reference system defined for each microphone pair
    % Multiple sources parameters
    nsrce                      = 10;         % Number of sources to be detected
    minAngle                   = 10;         % Minimum angle between peaks
    % Moving sources parameters
    blockDuration_sec          = [];         % Block duration in seconds (default []: one block for the whole signal)
    blockOverlap_percent       = [];         % Requested block overlap in percent (default []: No overlap) - is internally rounded to suited values
    % Wiener filtering
    enableWienerFiltering      = 0;          % 1: Process a Wiener filtering step in order to attenuate / emphasize the provided excerpt signal into the mixture signal. 0: Disable Wiener filtering
    wienerMode                 = [];         % Wiener filtering mode {'[]' 'Attenuation' 'Emphasis'}
    wienerRefSignal            = [];         % Excerpt of the source(s) to be emphasized or attenuated
    % Display results
    specDisplay                = 0;          % 1: Display angular spectrum found and sources directions found - 0: No display
    % Other parameters
    speedOfSound               = 343;        % Speed of sound (m.s-1) - typical value: 343 m.s-1 (assuming 20℃ in the air at sea level)
    fftSize_sec                = [];         % FFT size in seconds (default []: 0.064 sec)
    freqRange                  = [];         % Frequency range (pair of values in Hertz) to aggregate the angular spectrum : [] means that all frequencies will be used
    % Debug
    angularSpectrumDebug       = 0;          % 1: Enable additional plots to debug the angular spectrum aggregation

    %% Run the localization
    % Convert algorithm parameters to MBSS structures
    sMBSSParam = MBSS_InputParam2Struct(angularSpectrumMeth,speedOfSound,fftSize_sec,blockDuration_sec,blockOverlap_percent,pooling,azBound,elBound,gridRes,alphaRes,minAngle,nsrce,fs,applySpecInstNormalization,specDisplay,enableWienerFiltering,wienerMode,freqRange,micPos,isArrayMoving,subArray,sceneTimeStamps,angularSpectrumDebug);
    [azEst, elEst, ~, ~] = MBSS_locate_spec(x,wienerRefSignal,sMBSSParam); % Perform localization
    DOAs.azi = azEst;
    DOAs.ele = elEst;

end
