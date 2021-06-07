#include <vector>
#include <iostream>
#include <math.h>

using namespace std;
	class Fit{
		vector<double> factor;      //拟合后的方程系数
		double ssr;                 //回归平方和
		double sse;                 //残差平方和
		double rmse;                //RMSE均方根误差
		vector<double> fitedYs;     //存放拟合后的y值，在拟合时可设置为不保存节省内存
	public:
		Fit():ssr(0),sse(0),rmse(0){factor.resize(2,0);}
		~Fit(){}

		/* 多项式拟合，拟合y=a0+a1*x+a2*x^2+……+apoly_n*x^poly_n
		 x 观察值的x
		 y 观察值的y
		 poly_n 期望拟合的阶数，若poly_n=2，则y=a0+a1*x+a2*x^2
		 isSaveFitYs 拟合后的数据是否保存，默认是*/

		template<class T>
		void polyfit(const vector<T>& x,const vector<T>& y,int poly_n,bool isSaveFitYs=true)
		{
			polyfit(&x[0],&y[0],getSeriesLength(x,y),poly_n,isSaveFitYs);
		}
		template<class T>
		void polyfit(const T* x,const T* y,size_t length,int poly_n,bool isSaveFitYs=true)
		{
			factor.resize(poly_n+1,0);
			int i,j;
			vector<double> tempx(length,1.0);

			vector<double> tempy(y,y+length);

			vector<double> sumxx(poly_n*2+1);
			vector<double> ata((poly_n+1)*(poly_n+1));
			vector<double> sumxy(poly_n+1);
			for (i=0;i<2*poly_n+1;i++){
				for (sumxx[i]=0,j=0;j<length;j++)
				{
					sumxx[i]+=tempx[j];
					tempx[j]*=x[j];
				}
			}
			for (i=0;i<poly_n+1;i++){
				for (sumxy[i]=0,j=0;j<length;j++)
				{
					sumxy[i]+=tempy[j];
					tempy[j]*=x[j];
				}
			}
			for (i=0;i<poly_n+1;i++)
				for (j=0;j<poly_n+1;j++)
					ata[i*(poly_n+1)+j]=sumxx[i+j];
			gauss_solve(poly_n+1,ata,factor,sumxy);

			//计算拟合后的数据并计算误差
			fitedYs.reserve(length);
			calcError(&x[0],&y[0],length,this->ssr,this->sse,this->rmse,isSaveFitYs);
		}

		//获取拟合后系数
		void getFactor(std::vector<double>& factor){  factor = this->factor;  }

		//获取拟合方程对应的y值,前提是拟合时设置isSaveFitYs为true
		void getFitedYs(std::vector<double>& fitedYs){ fitedYs = this->fitedYs; }

		//根据x获取拟合方程的y值
		template<class T>
		double getY(const T x) const
		{
			double ans(0);
			for (size_t i=0;i<factor.size();++i)
			{
				ans += factor[i]*pow((double)x,(int)i);
			}
			return ans;
		}

		//剩余平方和
		double getSSE(){ return sse; }

		//回归平方和
		double getSSR(){return ssr;}

		//均方根误差
		double getRMSE(){return rmse;}

		//确定系数，系数是0~1之间的数，是数理上判定拟合优度的一个量
		double getR_square(){return 1-(sse/(ssr+sse));}

		//获取两个vector的安全最小size
		template<class T>
		size_t getSeriesLength(const vector<T>& x,const vector<T>& y)
		{
			return (x.size() > y.size() ? y.size() : x.size());
		}

		//计算均值
		template <class T>
		static T Mean(const vector<T>& v)
		{
			return Mean(&v[0],v.size());
		}
		template <class T>
		static T Mean(const T* v,size_t length)
		{
			T total(0);
			for (size_t i=0;i<length;++i)
			{
				total += v[i];
			}
			return (total / length);
		}

		//拟合方程系数的个数
		size_t getFactorSize(){return factor.size();}

		//获取拟合方程的系数
		double getFactor(size_t i){return factor.at(i);}

	private:
		template<class T>
		void calcError(const T* x,const T* y,size_t length,double& r_ssr,double& r_sse,double& r_rmse,bool isSaveFitYs=true)
		{
			T mean_y = Mean<T>(y,length);
			T yi(0);
			fitedYs.reserve(length);
			for (int i=0; i<length; ++i)
			{
				yi = getY(x[i]);
				r_ssr += ((yi-mean_y)*(yi-mean_y));//计算回归平方和
				r_sse += ((yi-y[i])*(yi-y[i]));//残差平方和
				if (isSaveFitYs) //保存拟合的y值
				{
					fitedYs.push_back(double(yi));
				}
			}
			r_rmse = sqrt(r_sse/(double(length)));
		}


		//gauss_solve(poly_n+1,ata,factor,sumxy);
		//最小二乘求解,AX=b
		template<class T>
		void gauss_solve(int n,std::vector< T>& A,std::vector< T>& x,std::vector< T>& b)
		{
			gauss_solve(n,&A[0],&x[0],&b[0]);
		}
		template<class T>
		void gauss_solve(int n,T* A,T* x,T* b)
		{
			int i,j,k,r;
			double max;
			for (k=0;k<n-1;k++)
			{
				max=fabs(A[k*n+k]); /*find maxmum*/
				r=k;
				for (i=k+1;i<n-1;i++){
					if (max<fabs(A[i*n+i]))
					{
						max=fabs(A[i*n+i]);
						r=i;
					}
				}
				if (r!=k){
					for (i=0;i<n;i++)         /*change array:A[k]&A[r] */
					{
						max=A[k*n+i];
						A[k*n+i]=A[r*n+i];
						A[r*n+i]=max;
					}
				}
				max=b[k];                    /*change array:b[k]&b[r]     */
				b[k]=b[r];
				b[r]=max;
				for (i=k+1;i<n;i++)
				{
					for (j=k+1;j<n;j++)
						A[i*n+j]-=A[i*n+k]*A[k*n+j]/A[k*n+k];
					b[i]-=A[i*n+k]*b[k]/A[k*n+k];
				}
			}

			for (i=n-1;i>=0;x[i]/=A[i*n+i],i--)
				for (j=i+1,x[i]=b[i];j<n;j++)
					x[i]-=A[i*n+j]*x[j];
		}
	};
