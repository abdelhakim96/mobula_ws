// libgp - Gaussian process library for Machine Learning
// Copyright (c) 2013, Manuel Blum <mblum@informatik.uni-freiburg.de>
// All rights reserved.

// Modified by Mohit Mehndiratta <mohit005@e.ntu.edu.sg> 2019

#include "gp_sparse.h"

namespace libgp {
  
  const double log2pi = log(2*M_PI);

  SparseGaussianProcess::SparseGaussianProcess (size_t input_dim, std::string covf_def, size_t _max_points) : GaussianProcess(input_dim, covf_def)
  {
    max_points = _max_points;
  }
  
  SparseGaussianProcess::SparseGaussianProcess (const char * filename) : GaussianProcess(filename)
  {
    max_points = this->sampleset->size();
  }
  
  SparseGaussianProcess::SparseGaussianProcess (size_t _max_points, const GaussianProcess& gp) : GaussianProcess(gp)
  {
    max_points = _max_points;
  }

  SparseGaussianProcess::~SparseGaussianProcess () {}  
  
  void SparseGaussianProcess::sparsify()
  {
//    // initialize the required matrices
//    initialize(_mu_star, _sx_star2);
    SampleSet * sampleset_indpts = new SampleSet(*sampleset);

    /// compute big distance matrix to avoid re-computations
    Eigen::MatrixXd distances = Eigen::MatrixXd::Zero(sampleset->size(), sampleset->size());
    for(size_t i = 0; i < sampleset->size(); ++i) {
      for(size_t j = 0; j < sampleset->size(); ++j) {
        distances(i, j) = (sampleset->x(i) - sampleset->x(j)).norm();
      }
    }
//    std::cout<<"distances_old = \n"<<distances<<"\n";

    while(sampleset_indpts->size() > max_points) {
      int k = get_most_dense_point(input_dim, sampleset_indpts->size(), distances);
      /// sanity check
      if (k < 0)
          break;
      sampleset_indpts->remove(k);

      remove_column(distances, k);
      remove_row(distances, k);
    }
//    std::cout<<"distances_new = \n"<<distances<<"\n";

    // compute prior mean and covariance for induced points
    prior_meanVar(sampleset_indpts);

    // replace measurements with induced points
    delete sampleset;
    sampleset = sampleset_indpts;
  }

  int SparseGaussianProcess::get_most_dense_point(size_t D, size_t N,
                                                  const Eigen::MatrixXd& distances)
  {
    double min_dist = std::numeric_limits<double>::max();
    int denser = -1;

    for(size_t i = 0; i < N; ++i) {
      double dist = 0.;
      std::vector<double> neighbors(N);
      Eigen::VectorXd::Map(neighbors.data(), neighbors.size()) = distances.row(i);
      /// remove self distance
      neighbors.erase(neighbors.begin() + i);

      std::partial_sort(neighbors.begin(), neighbors.begin() + D, neighbors.end());

      for (size_t j = 0; j < D; j++) {
          dist += neighbors[j];
      }
      if (dist < min_dist) {
          min_dist = dist;
          denser = i;
      }
    }
    return denser;
  }

  void SparseGaussianProcess::prior_meanVar(SampleSet * _sampleset)
  {
    size_t n = sampleset->size();
    size_t nu = _sampleset->size();

    Eigen::MatrixXd Kmm(n, n), Kuu(nu, nu), Kmu(n, nu);
    std::tie(Kmm, Kuu, Kmu) = compute_KmmKuuKmu(sampleset, _sampleset);
//    std::cout<<"Kmm = "<<Kmm<<"\n";
//    std::cout<<"Kuu = "<<Kuu<<"\n";
//    std::cout<<"Kmu = "<<Kmu<<"\n";

    // muu = Kmu'*inv(Kmm + Sfm)*fmh
    muu.resize(nu);
    // Update alpha using target values
    this->GaussianProcess::update_alpha();
    muu = Kmu.transpose()*alpha;
//    std::cout<<"muu = "<<muu<<"\n";
    // alpha to be updated again with muu for prediction
    alpha_needs_update = true;

    Eigen::MatrixXd Q(nu, nu);
    // Q = Kmu'inv(L'L)*Kmu
    Q = Kmu.transpose()*(Kmm.selfadjointView<Eigen::Lower>().llt().solve(Kmu));
//    std::cout<<"Q = "<<Q<<"\n";

    Suu.setZero(nu, nu);
//    Suu = Kuu - Q.topLeftCorner(n, n);
    // Suu = Q, to be helpful later
    Suu = Q;
//    std::cout<<"Suu = "<<Suu<<"\n";

//    // TO BE: is it necessary to compute it here? : no need, since cf->loghyper_changed is true.
//    // Replace L with Kuu and perform cholesky factorization: L = chol(L)
//    L.setZero(n, n);
//    L.topLeftCorner(nu, nu) = Kuu;
//    L.topLeftCorner(nu, nu) = L.topLeftCorner(nu, nu).selfadjointView<Eigen::Lower>().llt().matrixL();
////    std::cout<<"L = "<<L.topLeftCorner(nu, nu)<<"\n";

  }

  std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>
    SparseGaussianProcess::compute_KmmKuuKmu(SampleSet * sampleset_m, SampleSet * sampleset_u)
  {
    size_t n = sampleset_m->size();
    size_t nu = sampleset_u->size();

    // compute kernel matrix (lower triangle)
    Eigen::MatrixXd Kmm(n, n);
    Kmm.setZero(n, n);
    Kmm = (L.topLeftCorner(sampleset->size(), sampleset->size())*L.topLeftCorner(sampleset->size(), sampleset->size()).transpose()).triangularView<Eigen::Lower>();

    // compute kernel matrix (lower triangle)
    Eigen::MatrixXd Kuu(nu, nu);
    Kuu.setZero(nu, nu);
    for(size_t i = 0; i < nu; ++i) {
      for(size_t j = 0; j <= i; ++j) {
        (Kuu)(i, j) = cf->get(sampleset_u->x(i), sampleset_u->x(j));
      }
    }

    // compute kernel matrix
    Eigen::MatrixXd Kmu(n, nu);
    Kmu.setZero(n, nu);
    for(size_t i = 0; i < n; ++i) {
      for(size_t j = 0; j < nu; ++j) {
        // TO BE: check if the values for noise covariance is coming up in the below evaluation
        (Kmu)(i, j) = cf->get(sampleset_m->x(i), sampleset_u->x(j));
      }
    }

    return std::make_tuple(Kmm, Kuu, Kmu);
  }

  void SparseGaussianProcess::update_alpha()
  {
//    std::cout<<"alpha = "<<alpha<<"\n";
    // can previously computed values be used?
    if (!alpha_needs_update) return;
    alpha_needs_update = false;
    alpha.resize(sampleset->size());
    int n = sampleset->size();
//    std::cout<<"L = "<<L.topLeftCorner(n, n)<<"\n";
    // alpha = inv(L)*muu
    alpha = L.topLeftCorner(n, n).triangularView<Eigen::Lower>().solve(muu);
    // alpha = inv(L')*alpha
    // TO BE: check which one is correct? Top one is the original one
    L.topLeftCorner(n, n).triangularView<Eigen::Lower>().adjoint().solveInPlace(alpha); // Seems wrong!!!
//    L.topLeftCorner(n, n).triangularView<Eigen::Lower>().transpose().solveInPlace(alpha);
//    std::cout<<"alpha = "<<alpha<<"\n";
  }

  double SparseGaussianProcess::var(const double x[])
  {
    if (sampleset->empty()) return 0;
    Eigen::Map<const Eigen::VectorXd> x_star(x, input_dim);
    compute();
    update_alpha();
    update_k_star(x_star);
//    std::cout<<"k_star = "<<k_star<<"\n";
    int n = sampleset->size();
//    std::cout<<"L = "<<L.topLeftCorner(n, n)<<"\n";
    // v = inv(L)*k_star
    Eigen::VectorXd v = L.topLeftCorner(n, n).triangularView<Eigen::Lower>().solve(k_star);
    // v = inv(L')*v
    L.topLeftCorner(n, n).triangularView<Eigen::Lower>().transpose().solveInPlace(v);
//    std::cout<<"v = "<<v<<"\n";
//    std::cout<<"v.dot(Suu*v) = "<<v.dot(Suu*v)<<"\n";
//    std::cout<<"k_star_star = "<<cf->get(x_star, x_star)<<"\n";
    // var_f_star = k_star_star - v'*(Kuu - Suu)*v
    return cf->get(x_star, x_star) - v.dot(Suu*v);
  }
  
  std::pair<double, double> SparseGaussianProcess::f_var(const double x[])
  {
    if (sampleset->empty()) return std::make_pair(0, 0);
    Eigen::Map<const Eigen::VectorXd> x_star(x, input_dim);
    compute();
    update_alpha();
    update_k_star(x_star);
    int n = sampleset->size();
    // v = inv(L)*k_star
    Eigen::VectorXd v = L.topLeftCorner(n, n).triangularView<Eigen::Lower>().solve(k_star);
    // v = inv(L')*v
    L.topLeftCorner(n, n).triangularView<Eigen::Lower>().transpose().solveInPlace(v);

    return std::make_pair(k_star.dot(alpha), cf->get(x_star, x_star) - v.dot(Suu*v));
  }

//  void SparseGaussianProcess::initialize(Eigen::VectorXd &_mu_star, Eigen::VectorXd &_sx_star2)
//  {
//    // prior mean and variance of the induced points distribution
//    mu_star = _mu_star;
//    sx_star2 = _sx_star2;

//    ell2.setZero(input_dim);
//    if (cf->get_param_dim()-2 > 1)
//    {
//      for(size_t i = 0; i < input_dim; ++i)
//        ell2(i) = exp(2*(cf->get_loghyper())(i));
//      sqrt_det_ell2_by_det_ell2_p_2sx2 = sqrt((ell2.prod())/(ell2+2*sx_star2).prod());
//    }
//    else
//    {
//      for(size_t i = 0; i < input_dim; ++i)
//        ell2(i) = exp(2*(cf->get_loghyper())(0));
//      sqrt_det_ell2_by_det_ell2_p_2sx2 = sqrt((ell2(0))/(ell2(0)+2*sx_star2(0)));
//    }
//    sf2 = exp(2*(cf->get_loghyper())(cf->get_param_dim()-2));

//    Kmm.resize(initial_L_size, initial_L_size);
//    Kmm = (L.topLeftCorner(sampleset->size(), sampleset->size())*L.topLeftCorner(sampleset->size(), sampleset->size()).transpose()).triangularView<Eigen::Lower>();
//  }

//  double SparseGaussianProcess::expected_post_var(SampleSet *& _sampleset)
//  {
//    size_t n = sampleset->size();
//    size_t nu = _sampleset->size();

//    Eigen::MatrixXd Kuu(nu, nu), Kmu(n, nu);
//    std::tie(Kuu, Kmu) = compute_KuuKmu(sampleset, _sampleset);

//    Eigen::MatrixXd P(nu, nu), Q = Eigen::MatrixXd::Identity(n, n);
//    // compute Q =  Kmu*inv(Kuu)
//    Kuu.selfadjointView<Eigen::Lower>().llt().solveInPlace(Q);
//    Q = Kmu*Q;
////    std::cout<<"Q = "<<Q<<"\n";

//    // compute P =  Q'*inv(Kmm + Sfm)*Q
//    P = Q.transpose()*Kmm.selfadjointView<Eigen::Lower>().llt().solve(Q);
////    std::cout<<"P = "<<P<<"\n";

//    double sum = 0;
//    for(size_t i = 0; i < nu; ++i)
//    {
//      for(size_t j = 0; j <= i; ++j)
//      {
//        if (i==j)
//          sum = sum + P(i,j)*exp(-0.5*(_sampleset->x(i)-_sampleset->x(j))
//                                     .cwiseQuotient(2*ell2)
//                                     .dot(_sampleset->x(i)-_sampleset->x(j)))
//                            *exp(-0.5*(0.5*(_sampleset->x(i)+_sampleset->x(j))-mu_star)
//                                     .cwiseQuotient(0.5*ell2+sx_star2)
//                                     .dot(0.5*(_sampleset->x(i)+_sampleset->x(j))-mu_star));
//        else
//          sum = sum + 2*P(i,j)*exp(-0.5*(_sampleset->x(i)-_sampleset->x(j))
//                                       .cwiseQuotient(2*ell2)
//                                       .dot(_sampleset->x(i)-_sampleset->x(j)))
//                              *exp(-0.5*(0.5*(_sampleset->x(i)+_sampleset->x(j))-mu_star)
//                                       .cwiseQuotient(0.5*ell2+sx_star2)
//                                       .dot(0.5*(_sampleset->x(i)+_sampleset->x(j))-mu_star));
//      }
//    }
//    return sf2 - sf2*sf2*sqrt_det_ell2_by_det_ell2_p_2sx2*sum;
//  }

  /** add sample and update the GP. If the number of samples is bigger than
   *  the desired maximum points, we re-sparsify and re-compute the GP */
  void SparseGaussianProcess::add_pattern(const double x[], double y)
  {
    this->GaussianProcess::add_pattern(x, y);
//    std::cout<<"L = "<<L.topLeftCorner(sampleset->size(), sampleset->size())<<"\n";

    // if we surpassed the maximum points, re-sparsify and recompute
    if (sampleset->size() > max_points){
      sparsify();
    }

  }

  /// remove row from an Eigen matrix
  void SparseGaussianProcess::remove_row(Eigen::MatrixXd& matrix, unsigned int rowToRemove)
  {
      unsigned int numRows = matrix.rows() - 1;
      unsigned int numCols = matrix.cols();

      if (rowToRemove < numRows)
          matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) = matrix.block(rowToRemove + 1, 0, numRows - rowToRemove, numCols);

      matrix.conservativeResize(numRows, numCols);
  }

  /// remove column from an Eigen matrix
  void SparseGaussianProcess::remove_column(Eigen::MatrixXd& matrix, unsigned int colToRemove)
  {
      unsigned int numRows = matrix.rows();
      unsigned int numCols = matrix.cols() - 1;

      if (colToRemove < numCols)
          matrix.block(0, colToRemove, numRows, numCols - colToRemove) = matrix.block(0, colToRemove + 1, numRows, numCols - colToRemove);

      matrix.conservativeResize(numRows, numCols);
  }

  size_t SparseGaussianProcess::get_max_points()
  {
    return max_points;
  }

  void SparseGaussianProcess::set_max_points(size_t _max_points)
  {
    max_points = _max_points;
  }

}
