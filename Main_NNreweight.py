# STAT59800 Final Project Checkpoint 03

# load packages
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import os
os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"

def get_random_biased_dataset(train_ratio=0.95,test_ratio=0.5,Ntrain=1000,Nvalid=10,Ntest=100,seed=None,class0=4,class1=9):
    
    # fix the random seed for the fair comparison with baseline
    generator   =   np.random.seed(seed)
    
    # use imbalanced training data set / balanced test data set
    (train_images,train_labels),(test_images,test_labels)   =   tf.keras.datasets.mnist.load_data()
    train_images_class0     =   train_images[train_labels==class0]
    test_images_class0      =   test_images[test_labels==class0]
    train_images_class1     =   train_images[train_labels==class1]
    test_images_class1      =   test_images[test_labels==class1]
    
    # dtype conversion
    train_images_class0     =   train_images_class0.astype(np.float32)/255
    test_images_class0      =   test_images_class0.astype(np.float32)/255
    train_images_class1     =   train_images_class1.astype(np.float32)/255
    test_images_class1      =   test_images_class1.astype(np.float32)/255
    
    # random selection
    print('Randomly select %d train / %d validation / %d test data in class %d' % (int(np.floor(train_ratio*Ntrain)),int(np.floor(test_ratio*Nvalid)),int(np.floor(test_ratio*Ntest)),class0))
    train_images_class0_sel     =   train_images_class0[np.random.randint(0,train_images_class0.shape[0],int(np.floor(train_ratio*Ntrain))),:,:]
    valid_images_class0_sel     =   train_images_class0[np.random.randint(0,train_images_class0.shape[0],int(np.floor(test_ratio*Nvalid))),:,:]
    test_images_class0_sel      =   test_images_class0[np.random.randint(0,test_images_class0.shape[0],int(np.floor(test_ratio*Ntest))),:,:]

    print('Randomly select %d train / %d validation / %d test data in class %d' % (int(Ntrain-np.floor(train_ratio*Ntrain)),int(Nvalid-np.floor(test_ratio*Nvalid)),Ntest-int(np.floor(test_ratio*Ntest)),class1))
    train_images_class1_sel     =   train_images_class1[np.random.randint(0,train_images_class1.shape[0],int(Ntrain-np.floor(train_ratio*Ntrain))),:,:]
    valid_images_class1_sel     =   train_images_class1[np.random.randint(0,train_images_class1.shape[0],int(Nvalid-np.floor(test_ratio*Nvalid))),:,:]
    test_images_class1_sel      =   test_images_class1[np.random.randint(0,test_images_class1.shape[0],int(Ntest-np.floor(test_ratio*Ntest))),:,:]
    
    # concatenate class 0 and class 1 data
    x_train_concatenate         =   np.concatenate([train_images_class0_sel,train_images_class1_sel],axis=0)
    x_valid_concatenate         =   np.concatenate([valid_images_class0_sel,valid_images_class1_sel],axis=0)
    x_test_concatenate          =   np.concatenate([test_images_class0_sel,test_images_class1_sel],axis=0)

    y_train_concatenate         =   np.concatenate([np.zeros([train_images_class0_sel.shape[0]]),np.ones([train_images_class1_sel.shape[0]])])
    y_valid_concatenate         =   np.concatenate([np.zeros([valid_images_class0_sel.shape[0]]),np.ones([valid_images_class1_sel.shape[0]])])
    y_test_concatenate          =   np.concatenate([np.zeros([test_images_class0_sel.shape[0]]),np.ones([test_images_class1_sel.shape[0]])])

    # reshape for CONV2D
    x_train_concatenate         =   x_train_concatenate.reshape(-1,x_train_concatenate.shape[1],x_train_concatenate.shape[2],1)
    x_valid_concatenate         =   x_valid_concatenate.reshape(-1,x_valid_concatenate.shape[1],x_valid_concatenate.shape[2],1)
    x_test_concatenate          =   x_test_concatenate.reshape(-1,x_test_concatenate.shape[1],x_test_concatenate.shape[2],1)

    # shuffle for training
    idx     =   np.arange(x_train_concatenate.shape[0])
    np.random.shuffle(idx)
    x_train         =   x_train_concatenate[idx]
    y_train         =   y_train_concatenate[idx]

    idx     =   np.arange(x_valid_concatenate.shape[0])
    np.random.shuffle(idx)
    x_valid         =   x_valid_concatenate[idx]
    y_valid         =   y_valid_concatenate[idx]

    idx     =   np.arange(x_test_concatenate.shape[0])
    np.random.shuffle(idx)
    x_test          =   x_test_concatenate[idx]
    y_test          =   y_test_concatenate[idx]

    return x_train, y_train, x_valid, y_valid, x_test, y_test

def LeNet_model(inputs,labels,theta_dict=None,w=None,reuse=None,dtype=tf.float32):
    
    # check whether the parameters of the nueral network is specified, if not specified initialize with empty dictionary
    if theta_dict is None:
        theta_dict  =   {}

    def _get_var(name,shape,dtype,initializer):
        key     =   tf.get_variable_scope().name + '/' + name
        
        # check whether the given variable is created
        if key in theta_dict:
            return theta_dict[key]
        else:
            # create a new variable
            var             =   tf.get_variable(name,shape,dtype,initializer=initializer)
            theta_dict[key] =   var
            return var

    with tf.variable_scope('LeNet',reuse=reuse):
        inputs_     =   tf.cast(tf.reshape(inputs,[-1,28,28,1]),dtype)
        labels      =   tf.cast(labels,dtype)

        # create weight
        theta_init  =   tf.truncated_normal_initializer(mean=0,stddev=0.1)
        theta1      =   _get_var('theta1',[5,5,1,16],dtype,initializer=theta_init)
        theta2      =   _get_var('theta2',[5,5,16,32],dtype,initializer=theta_init)
        theta3      =   _get_var('theta3',[5,5,32,64],dtype,initializer=theta_init)
        theta4      =   _get_var('theta4',[1024,100],dtype,initializer=theta_init)
        theta5      =   _get_var('theta5',[100,1], dtype,initializer=theta_init)

        # create offset
        b_init      =   tf.constant_initializer(0.0)
        b1          =   _get_var('b1',[16],dtype,initializer=b_init)
        b2          =   _get_var('b2',[32],dtype,initializer=b_init)
        b3          =   _get_var('b3',[64],dtype,initializer=b_init)
        b4          =   _get_var('b4',[100],dtype,initializer=b_init)
        b5          =   _get_var('b5',[1],dtype,initializer=b_init)
        
        # create layers
        # convolution layer level 1
        #l0      =   tf.identity(inputs_,name='l0')
        z1  =   tf.add(tf.nn.conv2d(inputs_,theta1,[1,1,1,1],'SAME'),b1,name='z1')
        l1  =   tf.nn.relu(tf.nn.max_pool(z1,[1,3,3,1],[1,2,2,1],'SAME'),name='l1')

        # convolution layer level 2
        z2  =   tf.add(tf.nn.conv2d(l1,theta2,[1,1,1,1],'SAME'),b2,name='i2')
        l2  =   tf.nn.relu(tf.nn.max_pool(z2,[1,3,3,1],[1,2,2,1],'SAME'),name='l2')

        # convolution layer level 3
        z3  =   tf.add(tf.nn.conv2d(l2,theta3,[1,1,1,1],'SAME'),b3,name='z3')
        l3  =   tf.nn.relu(tf.nn.max_pool(z3,[1,3,3,1],[1,2,2,1],'SAME'),name='l3')

        # fully connected layer level 4
        z4  =   tf.add(tf.matmul(tf.reshape(l3,[-1,1024]),theta4),b4,name='z4')
        l4  =   tf.nn.relu(z4,name='l4')

        # FC-5
        z5  =   tf.add(tf.matmul(l4,theta5),b5,name='z5')

        logits  =   tf.squeeze(z5)
        out     =   tf.sigmoid(logits)
        
        # multiply sample weight
        if w is None:
            # average loss
            loss    =   tf.reduce_mean(tf.nn.sigmoid_cross_entropy_with_logits(logits=logits,labels=labels))
        else:
            # weighted loss
            loss    =   tf.reduce_sum(tf.nn.sigmoid_cross_entropy_with_logits(logits=logits,labels=labels)*w)
    return theta_dict, loss, logits

def reweight_autodiff(input_bias,label_bias,input_valid,label_valid,bsize_bias,bsize_valid,eps=0.0,gate_gradients=1):

    w_bias      =   tf.zeros([bsize_bias],dtype=tf.float32)
    w_valid     =   tf.ones([bsize_valid],dtype=tf.float32)/float(bsize_valid)
    
    [theta_dict,loss_bias,logits_bias] = LeNet_model(input_bias,label_bias,w=w_bias,reuse=True)
    var_names   =   theta_dict.keys()
    var_list    =   [theta_dict[idx] for idx in var_names]
    grads       =   tf.gradients(loss_bias,var_list,gate_gradients=gate_gradients)

    var_list_new    =   [vv-gg for gg,vv in zip(grads,var_list)]
    theta_dict_new  =   dict(zip(var_names,var_list_new))
    [_,loss_valid,logits_valid] = LeNet_model(input_valid,label_valid,theta_dict=theta_dict_new,w=w_valid,reuse=True)
    grads_w         =   tf.gradients(loss_valid,[w_bias],gate_gradients=gate_gradients)[0]
    ex_weight       =   -grads_w
    ex_weight_plus  =   tf.maximum(ex_weight,eps)
    ex_weight_sum   =   tf.reduce_sum(ex_weight_plus)
    ex_weight_sum   +=  tf.to_float(tf.equal(ex_weight_sum, 0.0))
    ex_weight_norm  =   ex_weight_plus/ex_weight_sum
    
    return ex_weight_norm

def get_next_batch(x,y,Nepoch=0,cur_idx=0,shuffle=False,bsize=100):
    
    if (Nepoch == 0) and (cur_idx==0) and shuffle:
        idx     =   np.arange(x.shape[0])
        np.random.shuffle(idx)
        x       =   x[idx,:,:]
        y       =   y[idx]
        
    if cur_idx+bsize <= x.shape[0]:
        xbatch  =   x[cur_idx:cur_idx+bsize,:,:]
        ybatch  =   y[cur_idx:cur_idx+bsize]
        cur_idx =   cur_idx+bsize
        
    else:
        get_samples     =   x.shape[0]-cur_idx
        xbatch_temp     =   x[cur_idx:x.shape[0],:,:]
        ybatch_temp     =   y[cur_idx:x.shape[0]]
        
        if shuffle:
            idx     =   np.arange(x.shape[0])
            np.random.shuffle(idx)
            x       =   x[idx,:,:]
            y       =   y[idx]
            
        Nepoch          +=  1
        left_samples    =   bsize-get_samples
        xbatch          =   np.concatenate((xbatch_temp,x[0:left_samples,:,:]),axis=0)
        ybatch          =   np.concatenate((ybatch_temp,y[0:left_samples]),axis=0)
        cur_idx         =   left_samples
        
    return xbatch, ybatch, x, y, Nepoch, cur_idx    

def train_compare(mode,bias_ratio=0.90):
    
    # create imbalanced datatset
    # set the size of training/validation/test sets
    Ntrain  =   5000
    Nvalid  =   10
    Ntest   =   1000

    # adjust the ratio of trainig/validation/test samples
    train_ratio =   bias_ratio
    test_ratio  =   0.50

    # get the trainig/validation/test samples
    [x_train,y_train,x_valid,y_valid,x_test,y_test] =   get_random_biased_dataset(train_ratio,test_ratio,Ntrain,Nvalid,Ntest,seed=None,class0=4,class1=9)
        
    # reset tf graphs
    tf.reset_default_graph()
    
    # begin    
    with tf.Session() as sess:
        
        # set learning rate, batch size,
        lr      =   1e-3
        moment  =   0.9
        bsize   =   100
        
        # place holder for DNN model input/output
        x_      =   tf.placeholder(tf.float32,[None,x_train.shape[1],x_train.shape[2],1],name='x')
        y_      =   tf.placeholder(tf.float32,[None],name='y')
        x_val_  =   tf.placeholder(tf.float32,[None,x_valid.shape[1],x_valid.shape[2],1],name='x_val')
        y_val_  =   tf.placeholder(tf.float32,[None],name='y_val')
        w_      =   tf.placeholder(tf.float32,[None],name='w')
        lr_     =   tf.placeholder(tf.float32,[],name='lr')

        # build training model
        with tf.name_scope('Train'):
            [_,loss_c,logits_c]     =   LeNet_model(x_,y_,theta_dict=None,w=w_,reuse=None,dtype=tf.float32)
            train_op                =   tf.train.MomentumOptimizer(lr,moment).minimize(loss_c)

        # build evaluation model
        with tf.name_scope('Val'):
            [_,loss_eval,logits_eval]   =   LeNet_model(x_,y_,theta_dict=None,w=w_,reuse=True,dtype=tf.float32)
            prediction_                 =   tf.cast(tf.sigmoid(logits_eval)>0.5,tf.float32)
            acc_                        =   tf.reduce_mean(tf.cast(tf.equal(prediction_,y_),tf.float32))

        # build reweighting model
        w_values_   =   reweight_autodiff(x_, y_,x_val_,y_val_,bsize,min(bsize,Nvalid),eps=0.0,gate_gradients=1)
        
        # training process parameters
        Nepoch_train    =   0
        train_cur_idx   =   0
        Nepoch_valid    =   0
        valid_cur_idx   =   0
        
        # measure metric
        acc_train_sum   =   0.0
        acc_test_sum    =   0.0
        
        # train (5 epochs)
        iter_target     =   int(Ntrain/bsize*4)
        iter            =   0
        
        # result
        iter_hist       =   np.empty((0))
        train_acc_hist  =   np.empty((0))
        test_acc_hist   =   np.empty((0))
        train_loss_hist =   np.empty((0))
        
        sess.run(tf.global_variables_initializer())
        
        for iter in tqdm(range(iter_target)):
            [x_minibatch,y_minibatch,x_train,y_train,Nepoch_train,train_cur_idx]            =   get_next_batch(x_train,y_train,Nepoch_train,train_cur_idx,True,bsize)
            [x_val_minibatch,y_val_minibatch,x_valid,y_valid,Nepoch_valid,valid_cur_idx]    =   get_next_batch(x_valid,y_valid,Nepoch_valid,valid_cur_idx,True,min(bsize,Nvalid))

            if mode == 'baseline':
                [loss,acc,_]    =   sess.run([loss_c,acc_,train_op],feed_dict={x_:x_minibatch,y_:y_minibatch,x_val_:x_val_minibatch,y_val_:y_val_minibatch,w_:np.ones((bsize),dtype=np.float32)*np.float32(1/bsize),lr_:lr})
                
            else:
                w_value         =   sess.run(w_values_,feed_dict={x_:x_minibatch,y_:y_minibatch,x_val_:x_val_minibatch,y_val_:y_val_minibatch})
                [loss,acc,_]    =   sess.run([loss_c,acc_,train_op],feed_dict={x_:x_minibatch,y_:y_minibatch,x_val_:x_val_minibatch,y_val_:y_val_minibatch,w_:w_value,lr_:lr})

            if (iter) % 2 == 0:
                acc_train_sum   =   0.0
                acc_test_sum    =   0.0
                
                temp_train_cur_idx  =   0
                temp_test_cur_idx   =   0
                
                for step in range(x_train.shape[0]//bsize):
                    [x_temp_minibatch,y_temp_minibatch,_,_,_,temp_train_cur_idx]    =   get_next_batch(x_train,y_train,0,temp_train_cur_idx,True,bsize)
                    acc             =   sess.run(acc_,feed_dict={x_:x_temp_minibatch,y_:y_temp_minibatch})
                    acc_train_sum   +=  acc

                for step in range(x_test.shape[0]//bsize):
                    [x_test_minibatch,y_test_minibatch,_,_,_,temp_test_cur_idx]    =   get_next_batch(x_test,y_test,0,temp_test_cur_idx,True,bsize)
                    acc             =   sess.run(acc_,feed_dict={x_:x_test_minibatch,y_:y_test_minibatch})
                    acc_test_sum    +=  acc

                train_acc   =   acc_train_sum/float(x_train.shape[0]//bsize)
                test_acc    =   acc_test_sum/float(x_test.shape[0]//bsize)
                
                iter_hist       =   np.append(iter_hist,iter+1)
                train_acc_hist  =   np.append(train_acc_hist,train_acc)
                test_acc_hist   =   np.append(test_acc_hist,test_acc)
                train_loss_hist =   np.append(train_loss_hist,loss)
              
            if iter == iter_target-1:
                print('Method (',mode,'): Training Loss = ',train_loss_hist[-1],',  Training accuracy = ',test_acc_hist[-1], ',  Test accuracy = ',train_acc_hist[-1])
    
    return iter_hist, train_acc_hist, test_acc_hist

def main(args=None):
    
    test_mode           =   ('baseline','reweight')
    bias_ratio          =   np.array([0.5, 0.6, 0.7, 0.8, 0.9, 0.95],dtype=np.float32)
    result_iter         =   []
    result_train_acc    =   []
    result_test_acc     =   []
    
    for idx1 in range(bias_ratio.shape[0]):
        result_iter.append([])
        result_train_acc.append([])
        result_test_acc.append([])
        for idx2 in range(len(test_mode)):
            [iter_hist_temp,train_acc_hist_temp,test_acc_hist_temp]  =   train_compare(test_mode[idx2],bias_ratio[idx1])
            result_iter[idx1].append(iter_hist_temp)
            result_train_acc[idx1].append(train_acc_hist_temp)
            result_test_acc[idx1].append(test_acc_hist_temp)
        
    # plot iter - accruacy graph for both approaches
    [fig1,ax1]      =   plt.subplots()
    reweight1        =   ax1.plot(result_iter[4][1],result_test_acc[4][1],linewidth=2.5,label='Reweighting',zorder=0)   
    baseline1        =   ax1.plot(result_iter[4][0],result_test_acc[4][0],linewidth=2.5,label='Baseline (w/o reweighting)',zorder=1)
    ax1.grid()
    ax1.set_xlabel('Iteration [-]')
    ax1.set_ylabel('Accuracy (Test Set) [%]')
    plt.rcParams["legend.loc"] = 'lower right' 
    ax1.legend()
    plt.title("Accuracy vs Iteration (Imbalance 90%)")
    plt.savefig('iter_accuracy.png')
    
    # plot ratio - accruacy graph for both approaches
    final_accuracy_base     =   np.empty((0))
    final_accuracy_reweight =   np.empty((0))
    for idx1 in range(bias_ratio.shape[0]):
        final_accuracy_base     =   np.append(final_accuracy_base,result_test_acc[idx1][0][-1])
        final_accuracy_reweight =   np.append(final_accuracy_reweight,result_test_acc[idx1][1][-1])
    
    [fig2,ax2]      =   plt.subplots()
    reweight2       =   ax2.plot(bias_ratio,final_accuracy_reweight,linewidth=2.5,label='Reweighting',zorder=0)
    baseline2       =   ax2.plot(bias_ratio,final_accuracy_base,linewidth=2.5,label='Baseline (w/o reweighting)',zorder=1)
    ax2.grid()
    ax2.set_xlabel('Class imbalance ratio [%]')
    ax2.set_ylabel('Accuracy (Test Set) [%]')
    plt.rcParams["legend.loc"] = 'lower right' 
    ax2.legend()
    plt.title("Accuracy vs Imbalance ratio (5 epoch)")
    plt.savefig('imbal_accuracy.png')
    
    plt.show()
        
if __name__ == '__main__':
    main()