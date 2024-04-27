clear all
close all
clc

load Residual_Training_Choice_1.mat
load Residual_Training_Label_Choice_1.mat

XTrain = DATA_Value;
YTrain = DATA_Label;

load Residual_Testing_Choice_1.mat
load Residual_Testing_Label_Choice_1.mat

XTest = DATA_Value;
YTest = DATA_Label;

XTrain_new = zeros(2,length(XTrain{1}),1,length(XTrain));
XTest_new  = zeros(2,length(XTest{1}),1,length(XTest));

Motor_Fault = ["1","2","3","4"];

%% Data Preprocessing

% Training Data 
for i = 1 : length(XTrain)
    XTrain_new(:,:,:,i) = real(XTrain{i});
end
YTrain_new = cell2mat(YTrain');
YTrain_New = discretize(YTrain_new,[0.9 1.9 2.9 3.9 4.9],"categorical",Motor_Fault);

% Testing Data 
for i = 1 : length(XTest)
    XTest_new(:,:,:,i) = real(XTest{i});
end
YTest_new = cell2mat(YTest');
YTest_New = discretize(YTest_new,[0.9 1.9 2.9 3.9 4.9],"categorical",Motor_Fault);

Training_ratio   = 0.85;
Validating_ratio = 0.15;

numObservations             = size(XTrain_new,4);
[idxTrain,idxValidation]    = trainingPartitions(numObservations,[Training_ratio Validating_ratio]);

XValidation                 = XTrain_new(:,:,:,idxValidation);
YValidation                 = YTrain_New(idxValidation);

XTrain_new                  = XTrain_new(:,:,:,idxTrain);
YTrain_New                  = YTrain_New(idxTrain);

numResponses                = 4;

Neuron1                     = 10;
Neuron2                     = 10;

%% Define Neural Network Architecture (뉴럴네트워크 설계 과정)
layers = [
    
    imageInputLayer([2 length(XTrain{1}) 1])
    
    % Hiden Layer 1
    convolution2dLayer(2,Neuron1,Padding="same")
    batchNormalizationLayer
    reluLayer
    averagePooling2dLayer(2,Stride=2)                    % [-] Pooling layers reduce the dimension of an image
    
    % Hiden Layer 2
    convolution2dLayer(2,Neuron2,Padding="same")        
    batchNormalizationLayer
    reluLayer

    % Output Layer
    fullyConnectedLayer(numResponses)
    
    softmaxLayer];

miniBatchSize       = 10;
validationFrequency = floor(numel(YTrain_New)/miniBatchSize);

%% Specify Training Option (학습 파라미터 설정)
options = trainingOptions("sgdm", ...
    MiniBatchSize=miniBatchSize, ...
    InitialLearnRate=1e-3, ...
    LearnRateSchedule="piecewise", ...
    LearnRateDropFactor=0.1, ...
    LearnRateDropPeriod=20, ...
    Shuffle="every-epoch", ...
    ValidationData={XValidation,YValidation}, ...
    ValidationFrequency=validationFrequency, ...
    Plots="training-progress", ...
    Metrics="accuracy", ...
    Verbose=false);

%% Conduct Training & Testing
net         = trainnet(XTrain_new,YTrain_New,layers,"mse",options);
scores      = predict(net,XTest_new);
probability = [];

for i = 1 : length(scores)
    probability = [ probability; find(scores(i,:) == max(scores(i,:))) ];
end

%% Comparision
x    = 1:length(scores);
vals = [YTest_new';probability'];
b    = bar(x,vals);
legend('Test','Prediction')
xlabel('Test Data')
ylabel('Motor Fault')
grid on;

rho  = 1-(nnz(vals(1,:)-vals(2,:))/length(scores));
A1   = 100*rho;

sprintf('Motor Fault Detection Accuracy = %d Percent',A1)

