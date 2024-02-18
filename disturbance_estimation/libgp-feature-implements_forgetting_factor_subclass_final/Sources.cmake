SET(LIBGP_SRC
  src/cov.cc
  src/cov_factory.cc
  src/cov_linear_ard.cc
  src/cov_linear_one.cc
  src/cov_matern3_iso.cc
  src/cov_matern5_iso.cc
  src/cov_noise.cc
  src/cov_rq_iso.cc
  src/cov_periodic_matern3_iso.cc
  src/cov_periodic.cc
  src/cov_se_ard.cc
  src/cov_se_iso.cc
  src/cov_sum.cc
  src/cov_prod.cc
  src/gp.cc
  src/gp_forget_factor.cc
  src/gp_sparse.cc
  src/gp_sparse_forget_factor.cc
  src/gp_utils.cc
  src/sampleset.cc
  src/rprop.cc
  src/input_dim_filter.cc
  src/cg.cc
)

SET(LIBGP_INTERFACES 
  include/cov.h
  include/cov_factory.h
  include/cov_linear_ard.h
  include/cov_linear_one.h
  include/cov_matern3_iso.h
  include/cov_matern5_iso.h
  include/cov_noise.h
  include/cov_rq_iso.h
  include/cov_periodic_matern3_iso.h
  include/cov_periodic.h
  include/cov_se_ard.h
  include/cov_se_iso.h
  include/cov_sum.h
  include/cov_prod.h
  include/gp.h
  include/gp_forget_factor.h
  include/gp_sparse.h
  include/gp_sparse_forget_factor.h
  include/gp_utils.h
  include/sampleset.h
  include/rprop.h
  include/input_dim_filter.h
  include/cg.h
)
