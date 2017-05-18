/*
    Convert from ROS tf.transformations.py
    
    Editor : Kartik,Chen (17/02/14)

    Usage:
        var e_m = euler_fomr_quaternion([x,y,z,w]);

        document.write('roll  = '+e_m[0]*_Math.RAD2DEG+'<BR>');
        document.write('pitch = '+e_m[1]*_Math.RAD2DEG+'<BR>');
        document.write('yaw   = '+e_m[2]*_Math.RAD2DEG+'<BR>');

*/

var EPS = 4e-16;
var _AXES2TUPLE = {
    'sxyz': [0, 0, 0, 0], 'sxyx': [0, 0, 1, 0], 'sxzy': [0, 1, 0, 0],
    'sxzx': [0, 1, 1, 0], 'syzx': [1, 0, 0, 0], 'syzy': [1, 0, 1, 0],
    'syxz': [1, 1, 0, 0], 'syxy': [1, 1, 1, 0], 'szxy': [2, 0, 0, 0],
    'szxz': [2, 0, 1, 0], 'szyx': [2, 1, 0, 0], 'szyz': [2, 1, 1, 0],
    'rzyx': [0, 0, 0, 1], 'rxyx': [0, 0, 1, 1], 'ryzx': [0, 1, 0, 1],
    'rxzx': [0, 1, 1, 1], 'rxzy': [1, 0, 0, 1], 'ryzy': [1, 0, 1, 1],
    'rzxy': [1, 1, 0, 1], 'ryxy': [1, 1, 1, 1], 'ryxz': [2, 0, 0, 1],
    'rzxz': [2, 0, 1, 1], 'rxyz': [2, 1, 0, 1], 'rzyz': [2, 1, 1, 1]}

  // axis sequences for Euler angles
  var _NEXT_AXIS = [1, 2, 0, 1];

  function euler_from_matrix(matrix, axes='sxyz'){

      var firstaxis =_AXES2TUPLE[axes][0];
      var parity =_AXES2TUPLE[axes][1];
      var repetition =_AXES2TUPLE[axes][2];
      var frame =_AXES2TUPLE[axes][3];


      // try:
      //     firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
      // except (AttributeError, KeyError):
      //     _ = _TUPLE2AXES[axes]
      //     firstaxis, parity, repetition, frame = axes



      var i = firstaxis;
      var j = _NEXT_AXIS[i+parity];
      var k = _NEXT_AXIS[i-parity+1];

      
      var M = nj.array( [ [ matrix.get(0,0),matrix.get(0,1), matrix.get(0,2)],
                          [ matrix.get(1,0),matrix.get(1,1), matrix.get(1,2)],
                          [ matrix.get(2,0),matrix.get(2,1), matrix.get(2,2)] ]);
    
      var ax,ay,az;

      if(repetition){
          sy = Math.sqrt(M.get(i, j)*M.get(i, j) + M.get(i, k)*M.get(i, k));
          if(sy > EPS){
              ax = Math.atan2( M.get(i, j),  M.get(i, k));
              ay = Math.atan2( sy,       M.get(i, i));
              az = Math.atan2( M.get(j, i), -M.get(k, i));
          }else{
              ax = Math.atan2(-M.get(j, k),  M.get(j, j));
              ay = Math.atan2( sy,       M.get(i, i));
              az = 0.0;
          }
      }else{
          cy = Math.sqrt(M.get(i, i)*M.get(i, i) + M.get(j, i)*M.get(j, i))
          if (cy > EPS){
              ax = Math.atan2( M.get(k, j),  M.get(k, k));
              ay = Math.atan2(-M.get(k, i),  cy);
              az = Math.atan2( M.get(j, i),  M.get(i, i));
          }else{
              ax = Math.atan2(-M.get(j, k),  M.get(j, j));
              ay = Math.atan2(-M.get(k, i),  cy);
              az = 0.0;
          }
      }
      if(parity){
          [ax, ay, az] = [-ax, -ay, -az];
      }
      if(frame){
          [ax, az] = [az, ax];
      }
      return [ax, ay, az];


  }

  function quaternion_matrix(quaternion){

      var q = nj.array(quaternion, dtype='float64').reshape(1,4);

      var nq = nj.dot(q, q.T);

      
      if(nq < EPS){
           return nj.identity(4);
      }
      var sqrt_2_div_nq = Math.sqrt(2.0 / nq.get(0,0));
      
      q = nj.multiply(q,sqrt_2_div_nq);
     

      var q_outer = nj.dot(q.T, q);

      var q_m = nj.array([
          [1.0-q_outer.get(1, 1)-q_outer.get(2, 2),     q_outer.get(0, 1)-q_outer.get(2, 3),     q_outer.get(0, 2)+q_outer.get(1, 3), 0.0],
          [    q_outer.get(0, 1)+q_outer.get(2, 3), 1.0-q_outer.get(0, 0)-q_outer.get(2, 2),     q_outer.get(1, 2)-q_outer.get(0, 3), 0.0],
          [    q_outer.get(0, 2)-q_outer.get(1, 3),     q_outer.get(1, 2)+q_outer.get(0, 3), 1.0-q_outer.get(0, 0)-q_outer.get(1, 1), 0.0],
          [                0.0,                 0.0,                 0.0, 1.0]
        ],  dtype='float64');


      return q_m;
  }

function euler_fomr_quaternion(quaternion){
    var q_m = quaternion_matrix(quaternion);
    return euler_from_matrix(q_m);
}


function quaternion_from_euler(ai, aj, ak, axes='sxyz'){
    
    var firstaxis =_AXES2TUPLE[axes][0];
    var parity =_AXES2TUPLE[axes][1];
    var repetition =_AXES2TUPLE[axes][2];
    var frame =_AXES2TUPLE[axes][3];

    // try:
    //     firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    // except (AttributeError, KeyError):
    //     _ = _TUPLE2AXES[axes]
    //     firstaxis, parity, repetition, frame = axes



    var i = firstaxis;
    var j = _NEXT_AXIS[i+parity];
    var k = _NEXT_AXIS[i-parity+1];

    

    if (frame){
        [ai, ak] = [ak, ai];
    }
    if(parity){
        aj = -aj;
    }

    ai /= 2.0;
    aj /= 2.0;
    ak /= 2.0;
    ci = Math.cos(ai);
    si = Math.sin(ai);
    cj = Math.cos(aj);
    sj = Math.sin(aj);
    ck = Math.cos(ak);
    sk = Math.sin(ak);
    cc = ci*ck;
    cs = ci*sk;
    sc = si*ck;
    ss = si*sk;

    var quaternion = [0.0, 0.0, 0.0, 0.0];
    if(repetition){
        quaternion[i] = cj*(cs + sc);
        quaternion[j] = sj*(cc + ss);
        quaternion[k] = sj*(cs - sc);
        quaternion[3] = cj*(cc - ss);
    }else{
        quaternion[i] = cj*sc - sj*cs;
        quaternion[j] = cj*ss + sj*cc;
        quaternion[k] = cj*cs - sj*sc;
        quaternion[3] = cj*cc + sj*ss;
    }
    if(parity){
        quaternion[j] *= -1;
    }

    return quaternion;
}

// function quaternion_from_euler(euler){
//     return quaternion_from_euler(euler[0], euler[1], euler[2]);
// }