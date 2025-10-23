# Capacity Reservation Model
$
\underset{l}{min}~\sum_{o,d}\sum_{a} l_{o,d}^a\\ 
\qquad (1-\alpha)l \geq \alpha k\\
\qquad \alpha \sum_{o,d} \sum_{a} l_{o,d}^a \geq (1-\alpha)(\sum_{s} (p_s D_s) - \sum_{o,d}\sum_{a}(k_{o,d}^a))\\
\qquad c_a \geq k_{o,d}^a
$

# Demand Routing Model
We want to minimize the number of trucks assigned and aircraft used as a function of distance 

$
\underset{l}{min}~\sum_{o,d} t_{o,d}(d_{o,d} + c_{time}s_t + c_{truck}) + \sum_{o,d}\sum_{a} u_{o,d}^{a}(d_{o,d} + c_{time}s_a + c_{a}) + c_{excess}e_d^{\mathcal{D}}\\ 
\qquad \sum_{o,d} t_{o,d} \leq T_{pool}\\
\qquad \sum_{o} x_{o,d} + e_{o}^\mathcal{S} = \mathcal{S}_o\\
\qquad \sum_{o} x_{o,d} + e_{d}^\mathcal{D} = \mathcal{D}_d\\
\qquad \sum_{j \neq i} z_{i,j}^{o,d} - \sum_{j\neq i}z_{j,i}^{o,d} = \begin{cases} x_{o,d}, & i=o\\ -x_{o,d}, & i=d\\ 0, & o/w\end{cases}\\
\qquad \sum_{d} z_{i,j}^{o,d} \leq t_{o,d}b^t + \sum_a u_{o,d}^a b^a
$

