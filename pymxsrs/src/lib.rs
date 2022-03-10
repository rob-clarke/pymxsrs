extern crate nalgebra as na;

use pyo3::prelude::*;

mod windmodel;
use windmodel::PyWindModel;

use mxsrs::MXS;
use mxsrs::aerso::types::*;

enum MXSImpl {
    NoWind(MXS<mxsrs::aerso::wind_models::ConstantWind<f64>>),
    CustomWind(MXS<windmodel::PyWindModel>),
}

#[pyclass(name="MXS",unsendable)]
struct PyMXS {
    mxs: MXSImpl,
}

#[pymethods]
impl PyMXS {
    
    #[getter]
    fn get_position(&self) -> PyResult<[f64;3]> {
        match &self.mxs {
            MXSImpl::NoWind(mxs) => Ok(mxs.0.body.position().into()),
            MXSImpl::CustomWind(mxs) => Ok(mxs.0.body.position().into()),
        }
    }
    
    #[getter]
    fn get_velocity(&self) -> PyResult<[f64;3]> {
        match &self.mxs {
            MXSImpl::NoWind(mxs) => Ok(mxs.0.body.velocity().into()),
            MXSImpl::CustomWind(mxs) => Ok(mxs.0.body.velocity().into()),
        }
    }
    
    #[getter]
    fn get_attitude(&self) -> PyResult<[f64;4]> {
        let q = match &self.mxs {
            MXSImpl::NoWind(mxs) => mxs.0.body.attitude(),
            MXSImpl::CustomWind(mxs) => mxs.0.body.attitude(),
        };
        Ok([q.i, q.j, q.k, q.w])
    }
    
    #[getter]
    fn get_rates(&self) -> PyResult<[f64;3]> {
        match &self.mxs {
            MXSImpl::NoWind(mxs) => Ok(mxs.0.body.rates().into()),
            MXSImpl::CustomWind(mxs) => Ok(mxs.0.body.rates().into()),
        }
    }
    
    #[getter]
    fn get_statevector(&self) -> PyResult<[f64;13]> {
        match &self.mxs {
            MXSImpl::NoWind(mxs) => Ok(mxs.0.body.statevector().into()),
            MXSImpl::CustomWind(mxs) => Ok(mxs.0.body.statevector().into()),
        }
    }
    
    #[setter]
    fn set_statevector(&mut self, state: [f64;13]) -> PyResult<()> {
        match &mut self.mxs {
            MXSImpl::NoWind(mxs) => mxs.0.body.body.set_state(state.into()),
            MXSImpl::CustomWind(mxs) => mxs.0.body.body.set_state(state.into()),
        }
        Ok(())
    }
    
    #[new]
    fn new(
        mass: f64, inertia_py: Vec<Vec<f64>>,
        position_py: Vec<f64>, velocity_py: Vec<f64>,
        attitude_py: Vec<f64>, rates_py: Vec<f64>,
        wind_model_py: Option<Py<PyAny>>
    ) -> PyResult<Self> {
        let inertia = mxsrs::aerso::types::Matrix3::new(
            inertia_py[0][0],inertia_py[1][0],inertia_py[2][0],
            inertia_py[0][1],inertia_py[1][1],inertia_py[2][1],
            inertia_py[0][2],inertia_py[1][2],inertia_py[2][2]
            );
        
        let initial_position = Vector3::new(position_py[0], position_py[1], position_py[2]);
        let initial_velocity = Vector3::new(velocity_py[0], velocity_py[1], velocity_py[2]);
        let initial_attitude = UnitQuaternion::from_quaternion(
            na::Quaternion::from_parts(
                attitude_py[3],
                Vector3::new(attitude_py[0],attitude_py[1],attitude_py[2]))
            );
        let initial_rates = Vector3::new(rates_py[0],    rates_py[1],    rates_py[2]);
        
        match wind_model_py {
            Some(pyobject) => {
                let wind_model = Python::with_gil(|py| pyobject.extract::<PyWindModel>(py))?;
                Ok(Self {
                    mxs: MXSImpl::CustomWind(
                        MXS::new_with_state_and_windmodel(initial_position, initial_velocity, initial_attitude, initial_rates, wind_model)
                    )
                })
            }
            None => {
                Ok(Self {
                    mxs: MXSImpl::NoWind(
                        MXS::new_with_state(initial_position, initial_velocity, initial_attitude, initial_rates)
                    )
                })
            }
        }
        
    }
    
    fn step(&mut self, delta_t: f64, inputstate: [f64;4]) {
        match &mut self.mxs {
            MXSImpl::NoWind(mxs) => mxs.0.step(delta_t, &inputstate),
            MXSImpl::CustomWind(mxs) => mxs.0.step(delta_t, &inputstate),
        }
    }
}

#[pymodule]
fn pymxsrs(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<PyMXS>()?;
    Ok(())
}