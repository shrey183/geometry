extern crate nalgebra as na;
extern crate rand;
use std::f64::INFINITY;
use std::f64::consts::PI;


use rand::Rng;


#[allow(unused_imports)]
use na::{Vector3, approx_eq, dot, Norm, Vector2};

#[allow(dead_code)]
fn print_vector(m: &Vector3<f64>) {
    println!("{:?}", m)
}

// Infinte Plane (this is not part of the geometry stack, but I implemented this as warm-up)
pub struct PlaneSurface{
	n: Vector3<f64>,
	b: f64,
}
#[allow(deprecated)]
impl PlaneSurface {
	
	// Print the vectors forming the plane surface for debugging
	fn print_plane(&self){
		println!("=========================================================");
		println!("Printing the Information about the Plane");
		println!("{}x+{}y+{}z + {} = 0", self.n.x, self.n.y,self.n.z, self.b);
		println!("=========================================================");	
	}

	fn point_on_plane(&self, v: Vector3<f64>) -> bool{
	// Point is on the plane if dot(v,n) + b =0
		let val = dot(&v, &self.n) + self.b;
		if approx_eq(&0.0f64, &val){
			return true;		
		}
		return false;
	}
	
}

// Oriented Plane
pub struct OrientedPlaneSurface{
	origin: Vector3<f64>,
	hx: Vector3<f64>,	
	hy: Vector3<f64>,
	hz: Vector3<f64>,
}

#[allow(non_snake_case)]
#[allow(deprecated)]
impl OrientedPlaneSurface {
	
	// Print the vectors forming the oriented plane surface for debugging
	fn print_plane(&self){
		println!("=========================================================");
		println!("Printing the Information about the Plane");
		println!("orgin = {:?}", &self.origin);
		println!("hx = {:?}", &self.hx);
		println!("hy = {:?}", &self.hy);
		println!("hz = {:?}", &self.hz);		
		println!("=========================================================");	
	}

	fn point_on_plane(&self, v: Vector3<f64>) -> bool{
		// If O' denotes the origin of our plane and O the fixed origin
		// then v lies on the plane iff dot((O'Hz), O'v)) where
		// O'v = OO' - Ov = OO' - v = origin - v
		// O'Hz = hz - origin
		
		let O_prime_Hz = self.hz - self.origin; 
		let O_prime_v = self.origin - v; 
		let val = dot(&O_prime_Hz, &O_prime_v);
		if approx_eq(&0.0f64, &val){
			return true;		
		}
		return false;
	}


	// Given a point in the global coordinate system, if it lies on the oriented plane, we want to find its 2d coordinates

	pub fn three_d_to_two_d(&self, v: Vector3<f64>) -> Vector2<f64>{

			// Default output in case the point is not on the plane
			let mut result = Vector2::new(INFINITY, INFINITY);

			if self.point_on_plane(v){

				let o_prime_v = v - self.origin;
				let r = o_prime_v.norm();

				let o_prime_hx = self.hx - self.origin;
				let o_prime_hx_norm = o_prime_hx.norm(); 

				let norm_prod = r * o_prime_hx_norm;
				let theta = (dot(&o_prime_hx, &o_prime_v)/(norm_prod)).acos();

				let v_x = r * theta.cos();
				let v_y = r * theta.sin(); 

				result = Vector2::new(v_x, v_y);
				return result; 

			}

			return result
	}

	// Given a point in the local coordinate system, we want to find its 3d coordinates


	pub fn two_d_to_three_d(&self, v: Vector2<f64>) -> Vector3<f64>{

			// We first compute O'v then we know that Ov = OO' + O'v
			let extend_v = Vector3::new(v.x, v.y, 0.0); 
			let o_prime_v = extend_v - self.origin;
		
			let o_prime_o = self.origin;
			let result = o_prime_o + o_prime_v;
			
			return result;
	}

	
}

#[allow(non_snake_case)]
pub struct RectangleBound{
	p: OrientedPlaneSurface,
	h_x: f64,
	h_y: f64,
}

#[allow(non_snake_case)]
#[allow(deprecated)]
#[allow(unused_parens)]
impl RectangleBound {

	// Print information about the rectangle
	fn print_rectangle(&self){
		println!("=========================================================");
		println!("Printing Information about the rectangle surface");
		println!("orgin = {:?}", &self.p.origin);
		println!("hx = {:?}", &self.p.hx);
		println!("hy = {:?}", &self.p.hy);
		println!("hz = {:?}", &self.p.hz);
		println!("h_x = {}", self.h_x);
		println!("h_y = {}", self.h_y);
		println!("=========================================================");
		
	}
	// Is point on the recatangular surface? 

	fn point_on_rectangle(&self, v:Vector3<f64>) -> bool{
 
			
		// If the point is on the Oriented Surface
		if self.p.point_on_plane(v) {
			//  Then we want to get the 2d representation of the point on the OrientedSurface
		
			let result = self.p.three_d_to_two_d(v);
			let v_x = result.x;
			let v_y = result.y; 
			
			if ((v_x <= self.h_x && v_x >= -self.h_x) && (v_y <= self.h_y && v_y >= -self.h_y)){
					return true;
				}
			}		
		
		return false;
	}


}

#[allow(non_snake_case)]
#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    	fn test_point_on_plane() {

		let origin1 = Vector3::new(1.0, 1.0, 1.0);
		let hz1 = Vector3::new(1.0, 1.0, 2.0); // shift the e3 basis vector
		let hy1 = Vector3::new(1.0, 2.0, 1.0); // shift the e2 basis vector
		let hx1 = Vector3::new(2.0, 1.0, 1.0); // shift the e1 basis vector
		let oriented1 = OrientedPlaneSurface {origin: origin1, hx : hx1, hy : hy1, hz: hz1};

		// We can check this method by generating 100 random points (x+1, y+1, 1) with x and y following uniform(0,1)
	
		let mut rng = rand::thread_rng();
	
		let mut flag = true; 
		for _i in 0..100{
			let mut x = 2.0 * rng.gen::<f64>(); 
			let mut y = 2.0 * rng.gen::<f64>();
			let mut point = Vector3::new(x, y, 1.0);
			flag = flag & oriented1.point_on_plane(point);
			if flag == false{
				println!("Test fails for points ({}, {}, 1)", x, y);
			}
		}
	        assert_eq!(flag, true);
    	}



    #[test]
    	fn test_point_on_rectangle() {

		// We can check this method by generating 100 random points (x+1, y+1, 1) with x and y following uniform(0,1)

		// Make the rectangle bound
		let origin1 = Vector3::new(1.0, 1.0, 1.0);
		let hz1 = Vector3::new(1.0, 1.0, 2.0); // shift the e3 basis vector
		let hy1 = Vector3::new(1.0, 2.0, 1.0); // shift the e2 basis vector
		let hx1 = Vector3::new(2.0, 1.0, 1.0); // shift the e1 basis vector
		let oriented1 = OrientedPlaneSurface {origin: origin1, hx : hx1, hy : hy1, hz: hz1};

		let h_x1 = 1.0;
		let h_y1 = 1.0; 
		let rectbound = RectangleBound {p: oriented1, h_x: h_x1, h_y: h_y1}; 
	
		let mut rng = rand::thread_rng();
	
		let mut flag = true; 
		for _i in 0..100{
			let mut x = 2.0 * rng.gen::<f64>(); 
			let mut y = 2.0 * rng.gen::<f64>();
			let mut point = Vector3::new(x, y, 1.0);
			flag = flag & rectbound.point_on_rectangle(point);
			if flag == false{
				println!("Test fails for points ({}, {}, 1)", x, y);
			}
		}
	        assert_eq!(flag, true);
    	}

}


#[allow(unused_mut)]
fn main() {

	// Try to initialize a plane with a standard basis vectors
	let n1 = Vector3::new(1.0, 2.0, 3.0);
	let b1 = 0.0;
	// x+2y+3z = 0
	let plane1 = PlaneSurface {n : n1, b : b1};
	// Print the Plane
	PlaneSurface::print_plane(&plane1);
	// Check if the origin lies on the plane
	let mut point = Vector3::new(0.0, 0.0, 0.0);
	let mut result = plane1.point_on_plane(point);
	println!("{}", result);


	// Initialized an oriented plane which is just a translation
	// with unit vectors 
	let origin1 = Vector3::new(1.0, 1.0, 1.0);
	let hz1 = Vector3::new(1.0, 1.0, 2.0); // shift the e3 basis vector
	let hy1 = Vector3::new(1.0, 2.0, 1.0); // shift the e2 basis vector
	let hx1 = Vector3::new(2.0, 1.0, 1.0); // shift the e1 basis vector
	let mut oriented1 = OrientedPlaneSurface {origin: origin1, hx : hx1, hy : hy1, hz: hz1};

	// Print the Oriented Plane
	OrientedPlaneSurface::print_plane(&oriented1);
	// Check if the (0,0,0) lies on the plane
	result = oriented1.point_on_plane(point);
	println!("Does the point (0, 0, 0) lie on the oriented plane {}", result);

	// Check if the (0,0,1) lies on the plane
	point = Vector3::new(0.0, 0.0, 1.0);
	result = oriented1.point_on_plane(point);
	println!("Does the point (0, 0, 1) lie on the oriented plane {}", result);

	// Define a unit box using the previous oriented surface
	let h_x1 = 1.0;
	let h_y1 = 1.0; 
	let mut rectbound = RectangleBound {p: oriented1, h_x: h_x1, h_y: h_y1}; 
	// Print information about the Rectangle Bound
	RectangleBound::print_rectangle(&rectbound);


	// Check if the point (1.5, 1, 1) lies in the rectangle bound 
	// It does since if you translate it back, you get (0.5, 0, 0) which lies in the unit square
	point = Vector3::new(1.5, 1.0, 1.0);
	result = rectbound.point_on_rectangle(point);
	println!("Does the point (1.5, 1.0, 1.0) lie on the rectangle bound {}", result);


	// Check if the point (2.5, 1, 1) lies in the rectangle bound
	// It does not since if you translate it back, you get (1.5, 0, 0) which does not lie in the unit square
	point = Vector3::new(2.5, 1.0, 1.0);
	result = rectbound.point_on_rectangle(point);
	println!("Does the point (2.5, 1, 1) lie on the rectangle bound {}", result);


	// We can try to make a rectangular bound with a slightly more non-trivial oriented surface
	let origin1 = Vector3::new(1.0, 1.0, 1.0); // translation by (1,1,1)
	let hz1 = Vector3::new(1.0, 1.0, 2.0); // shift the e3 basis vector
	let angle = PI/6.0; // 30 deg anti-clockwise rotation
	let hx1 = Vector3::new(1.0 + angle.cos(), 2.0 - angle.sin(), 1.0); // transform the e1 vector
	let hy1 = Vector3::new(2.0 + angle.sin(), 1.0 + angle.cos(), 1.0); // transform the e2 vector
	oriented1 = OrientedPlaneSurface {origin: origin1, hx : hx1, hy : hy1, hz: hz1};
	rectbound = RectangleBound {p: oriented1, h_x: h_x1, h_y: h_y1}; 


	// We take the point (1,1,0), transform it to (cos(theta) -sin(theta) + 1, cos(theta) + sin(theta) + 1 , 1)
	// This puts the point on the edge of the bounded surface 
	point = Vector3::new(angle.cos() - angle.sin() + 1.0, angle.cos() + angle.sin() + 1.0, 1.0);
	result = rectbound.point_on_rectangle(point);
	println!("Does the point {} lie on the rectangle bound {}",point, result);

	
}





