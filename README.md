# DMPC-Benchmark-Problems
This repository containts a collection of benchmark problems for constraint-coupled distributed model predictive control. The benchmark problems have the following structure:
\begin{subequations}\label{DMPC_constr_couled}
	\begin{align}
		&\underset{\mathbf{x}^{0:N_p},\mathbf{u}^{0:N_p-1}}{\min}\;\sum_{i \in \mathcal{I}}\left[J_i^f(\mathbf{x}_i^{N_p}) + \sum_{k=0}^{N_p-1}J_i(\mathbf{x}_i^k,\mathbf{u}_i^k)\right],\\
		\text{s.\,t. } &\mathbf{x}_i^{k+1} = \mathbf{A}_{i}\mathbf{x}_i^k + \mathbf{B}_{i}\mathbf{u}_i^k,\;\forall i \in \mathcal{I},  k=0,\dots,N_p -1,\\
		&\mathbf{x}_i^0 = \tilde{\mathbf{x}}(t_0),\;\forall i \in \mathcal{I},\\
		&\mathbf{x}_i^{k}\in\mathcal{X}_i\subset \vbody{R}{\mathbf{x}_i},\;\forall i \in \mathcal{I}, k=0,\dots,N_p,\\
		&\mathbf{u}_i^{k}\in\mathcal{U}_i\subset \vbody{R}{\mathbf{u}_i},\;\forall i \in \mathcal{I},\;k=0,\dots,N_p-1,\\
		&\sum_{i \in \mathcal{I}} \mathbf{R}_i \mathbf{u}_i^k \le \mathbf{r}_{\max}^{k},\;k=0,\dots,N_p-1.
	\end{align}
\end{subequations}
