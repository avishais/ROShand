
import tensorflow as tf
import numpy as np

# Export net to text file - this function must be called within the session
def export_net(W, B, x_max, x_min, activation, sess, path_file = './models/net.netxt'):
    f = open(path_file,'w')

    k = W.keys()
    n = int(len(k))
    f.write(str(n) + ' ' + str(activation) + ' ')
    
    # W's
    for i in range(n):
        sth = 'h' + str(i+1)
        w = sess.run(W[sth])
        f.write(str(w.shape[0]) + ' ' + str(w.shape[1]) + ' ')
        for j in range(w.shape[0]):
            for k in range(w.shape[1]):
                f.write(str(w[j,k]) + ' ')
    
    # b's
    for i in range(n):
        sth = 'b' + str(i+1)
        b = sess.run(B[sth])
        f.write(str(b.shape[0]) + ' ')
        for j in range(b.shape[0]):
                f.write(str(b[j]) + ' ')

    # x_max and x_min
    for i in range(len(x_max)):
        f.write(str(x_max[i]) + ' ')
    for i in range(len(x_min)):
        f.write(str(x_min[i]) + ' ')

    f.close()

def normz(x, x_max, x_min):

    for i in range(x.shape[1]):
        x[:,i] = (x[:,i]-x_min[i])/(x_max[i]-x_min[i])
    
    # x = (x-x_min)/(x_max-x_min)

    return x

def denormz(x, x_max, x_min):
    x = x.reshape(2,)
    for i in range(x.shape[0]):
        x[i] = x[i]*(x_max[i]-x_min[i]) + x_min[i]

    # x = x*(x_max-x_min) + x_min
    
    return x

def normzG(x, mu, sigma):
    
    for i in range(x.shape[1]):
        x[:,i] = (x[:,i]-mu[i])/sigma[i]
    
    # x = (x-x_min)/(x_max-x_min)

    return x

def denormzG(x, mu, sigma):
    # x = x.reshape(2,)
    for i in range(x.shape[0]):
        x[i] = x[i]*sigma[i] + mu[i]

    # x = x*(x_max-x_min) + x_min
    
    return x

# -----------------------------------------------------------------------

def next_batch(num, data, labels):
    '''
    Return a total of `num` random samples. 
    Similar to mnist.train.next_batch(num)
    '''
    idx = np.arange(0 , len(data))
    np.random.shuffle(idx)
    idx = idx[:num]
    data_shuffle = [data[ i] for i in idx]
    labels_shuffle = [labels[ i] for i in idx]

    return np.asarray(data_shuffle), np.asarray(labels_shuffle)

# Compute L2 Regularization
def computeReg(weights):
    regularizer = 0

    for i in range(1, len(weights)+1):
        sth = 'h' + str(i)
        regularizer = regularizer + tf.nn.l2_loss(weights[sth])

    return regularizer

# Initialize weight and bias matrices
def wNb(num_input, hidden_layers, num_output):
    weights = {}
    biases = {}
    h = hidden_layers
    h = np.insert(h, 0, num_input)

    # initializer = tf.contrib.layers.xavier_initializer()
    # w1 = tf.Variable(initializer(w1_shape))
    # b1 = tf.Varialbe(initializer(b1_shape))

    # Net
    for i in range(len(h)-1):
        sth = 'h' + str(i+1)
        weights.update({sth: tf.Variable(tf.random_normal([h[i], h[i+1]]))})
        # weights.update({sth: tf.Variable(tf.random_uniform([h[i], h[i+1]], -1.0 / np.sqrt(h[i]), 1.0 / np.sqrt(h[i+1])))})
        stb = 'b' + str(i+1)
        biases.update({stb: tf.Variable(tf.random_normal([h[i+1]]))})
        # biases.update({stb: tf.Variable(tf.zeros([h[i+1]]))})

    weights.update({'h' + str(len(h)): tf.Variable(tf.zeros([h[len(h)-1], num_output]))})
    biases.update({'b' + str(len(h)): tf.Variable(tf.zeros([num_output]))})
       
    return weights, biases

def activF(x, activation_index):
    if activation_index==1:
        return tf.nn.sigmoid(x)
    if activation_index==2:
        return tf.nn.relu(x)
    if activation_index==3:
        return tf.nn.tanh(x)
    if activation_index==4:
        return tf.nn.elu(x)


# Building the net
def neural_net(x, weights, biases, activation_index=1):
    # First hidden fully connected layer 
    layer = activF(tf.add(tf.matmul(x, weights['h1']), biases['b1']), activation_index)

    # Remaining hidden fully connected layer 
    for i in range(1, int(len(weights)-1)):
        sth = 'h' + str(i+1)
        stb = 'b' + str(i+1)
        layer = activF(tf.add(tf.matmul(layer, weights[sth]), biases[stb]), activation_index)

    layer = tf.matmul(layer, weights['h' + str(len(weights))]) + biases['b' + str(len(weights))]

    return layer

# Building the net with dripout
def neural_net_dropout(x, weights, biases, keep_prob, activation_index=1):
    # First hidden fully connected layer 
    layer = activF(tf.add(tf.matmul(x, weights['h1']), biases['b1']), activation_index)
    layer_drop = tf.nn.dropout(layer, keep_prob)

    # Remaining hidden fully connected layer 
    for i in range(1, int(len(weights)-1)):
        sth = 'h' + str(i+1)
        stb = 'b' + str(i+1)
        layer = activF(tf.add(tf.matmul(layer_drop, weights[sth]), biases[stb]), activation_index)
        layer_drop = tf.nn.dropout(layer, keep_prob)

    layer = tf.matmul(layer_drop, weights['h' + str(len(weights))]) + biases['b' + str(len(weights))]

    return layer