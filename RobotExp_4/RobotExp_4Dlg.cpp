
// RobotExp_4Dlg.cpp : 구현 파일
//

#include "stdafx.h"
#include "RobotExp_4.h"
#include "RobotExp_4Dlg.h"
#include "afxdialogex.h"
#include "DataType.h"
#include <math.h>


#include "SystemMemory.h"
#ifdef _DEBUG
#define new DEBUG_NEW
#endif
#define _CRT_SECURE_NO_WARNINGS

// 응용 프로그램 정보에 사용되는 CAboutDlg 대화 상자입니다.

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 대화 상자 데이터입니다.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

// 구현입니다.
protected:
	DECLARE_MESSAGE_MAP()
public:
	/*afx_msg void OnBnClickedButtonInit();
	afx_msg void OnBnClickedButtonForward();
	afx_msg void OnBnClickedButtonInverse();*/
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
	/*ON_BN_CLICKED(IDC_BUTTON_INIT, &CAboutDlg::OnBnClickedButtonInit)
	ON_BN_CLICKED(IDC_BUTTON_FORWARD, &CAboutDlg::OnBnClickedButtonForward)
	ON_BN_CLICKED(IDC_BUTTON_INVERSE, &CAboutDlg::OnBnClickedButtonInverse)*/
END_MESSAGE_MAP()


// CRobotExp_4Dlg 대화 상자



CRobotExp_4Dlg::CRobotExp_4Dlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_ROBOTEXP_4_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CRobotExp_4Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_EDIT_TAR_POS1, m_editTarPos1);
	DDX_Control(pDX, IDC_EDIT_TAR_POS2, m_editTarPos2);
	DDX_Control(pDX, IDC_EDIT_CUR_POS1, m_editCurPos1);
	DDX_Control(pDX, IDC_EDIT_CUR_POS2, m_editCurPos2);

	DDX_Control(pDX, IDC_EDIT_CUR_VEL, m_editCurVel);
	DDX_Control(pDX, IDC_EDIT_TAR_VEL, m_editTarVel);
	DDX_Control(pDX, IDC_EDIT_CUR_TORQ, m_editCurTorq);
	DDX_Control(pDX, IDC_EDIT_TAR_TORQ, m_editTarTorq);

	DDX_Control(pDX, IDC_EDIT_TARX, m_editTarX);
	DDX_Control(pDX, IDC_EDIT_TARY, m_editTarY);
	DDX_Control(pDX, IDC_EDIT_TARZ, m_editTarZ);

	DDX_Control(pDX, IDC_EDIT_CURX, m_editCurX);
	DDX_Control(pDX, IDC_EDIT_CURY, m_editCurY); //C로 바꿀 것
	DDX_Control(pDX, IDC_EDIT_CURZ, m_editCurZ);
	DDX_Control(pDX, IDC_PORT, m_ComboPort);
	DDX_Control(pDX, IDC_BAUDRATE, m_ComboBaud);
	DDX_Control(pDX, IDC_CHECK_OPEN, m_CheckOpen);
}

BEGIN_MESSAGE_MAP(CRobotExp_4Dlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON_FORWARD, &CRobotExp_4Dlg::OnBnClickedButtonForward)
	ON_BN_CLICKED(IDC_BUTTON_INIT, &CRobotExp_4Dlg::OnBnClickedButtonInit)
	ON_WM_TIMER()
	ON_WM_DESTROY()
	ON_BN_CLICKED(IDC_BUTTON_INVERSE, &CRobotExp_4Dlg::OnBnClickedButtonInverse)
	ON_BN_CLICKED(IDC_BUTTON_GRAPH, &CRobotExp_4Dlg::OnBnClickedButtonGraph)
	ON_CBN_DROPDOWN(IDC_PORT, &CRobotExp_4Dlg::OnCbnDropdownComboPort)
	ON_BN_CLICKED(IDC_CHECK_OPEN, &CRobotExp_4Dlg::OnBnClickedCheckOpen)
	ON_BN_CLICKED(IDC_BUTTON_SET, &CRobotExp_4Dlg::OnBnClickedButtonSet)
END_MESSAGE_MAP()


// CRobotExp_4Dlg 메시지 처리기

BOOL CRobotExp_4Dlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 시스템 메뉴에 "정보..." 메뉴 항목을 추가합니다.

	// IDM_ABOUTBOX는 시스템 명령 범위에 있어야 합니다.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 이 대화 상자의 아이콘을 설정합니다.  응용 프로그램의 주 창이 대화 상자가 아닐 경우에는
	//  프레임워크가 이 작업을 자동으로 수행합니다.
	SetIcon(m_hIcon, TRUE);			// 큰 아이콘을 설정합니다.
	SetIcon(m_hIcon, FALSE);		// 작은 아이콘을 설정합니다.

	// TODO: 여기에 추가 초기화 작업을 추가합니다.
	m_pGraphDlg = new CGraphDlg; 
	m_pGraphDlg->Create(IDD_GRAPH_DIALOG);


	//SetTimer(1, 100, NULL);

	SetTimer(1001, 33, NULL);

	_commWorker.SetPeriod(0.01);
	_commWorker.SetWork(CreateWork<CCommWork>("Comm1Work"));

	// EditBox Value Init //
	m_editTarPos1.SetWindowTextA("0.0");
	m_editTarPos2.SetWindowTextA("0.0");
	m_editTarX.SetWindowTextA("0.0");
	m_editTarY.SetWindowTextA("0.0"); 
	m_editTarZ.SetWindowTextA("0.0");

	m_editCurPos1.SetWindowTextA("0.0");
	m_editCurPos2.SetWindowTextA("0.0");
	m_editCurX.SetWindowTextA("0.0");
	m_editCurY.SetWindowTextA("0.0");
	m_editCurZ.SetWindowTextA("0.0");
	
	m_editCurVel.SetWindowTextA("0.0");
	m_editTarVel.SetWindowTextA("10");
	m_editCurTorq.SetWindowTextA("0.0");
	m_editTarTorq.SetWindowTextA("0.1");

	return TRUE;  // 포커스를 컨트롤에 설정하지 않으면 TRUE를 반환합니다.
}

void CRobotExp_4Dlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 대화 상자에 최소화 단추를 추가할 경우 아이콘을 그리려면
//  아래 코드가 필요합니다.  문서/뷰 모델을 사용하는 MFC 응용 프로그램의 경우에는
//  프레임워크에서 이 작업을 자동으로 수행합니다.

void CRobotExp_4Dlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 그리기를 위한 디바이스 컨텍스트입니다.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 클라이언트 사각형에서 아이콘을 가운데에 맞춥니다.
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 아이콘을 그립니다.
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// 사용자가 최소화된 창을 끄는 동안에 커서가 표시되도록 시스템에서
//  이 함수를 호출합니다.
HCURSOR CRobotExp_4Dlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CRobotExp_4Dlg::SolveForwardKinematics(double dAngle, double dAngle2, double* pdPos)
{	
	double theta1 = dAngle;      
	double theta2 = dAngle2;
	
	pdPos[0] = cos(theta1) * 1.0 + (( cos(theta1)*cos(theta2) ) - ( sin(theta1)*sin(theta2) ))*1.0;
	pdPos[1] = sin(theta1) * 1.0 + (( cos(theta1)*sin(theta2) ) + ( cos(theta2)*sin(theta1) ))*1.0;
	pdPos[2] = 0;
}


void CRobotExp_4Dlg::SolveInverseKinematics(double dX, double dY, double dZ, double* pdAngle)
{
	double s1, s2, c1, c2;

	c2 = (pow(dX,2) + pow(dY,2) - ( pow(1.0, 2) + pow(1.0, 2) )) / (2 * 1.0);
	s2 = sqrt(1 - pow(c2, 2));

	double M11, M12, M21, M22;

	M11 = 1 + c2;	M12 = -s2;
	M21 = s2;		M22 = 1 + c2;

	c1 = (M22 * dX + -M12 * dY)  / (M11*M22 - M12*M21);
	s1 = (-M21 * dX + M11 * dY) / (M11*M22 - M12*M21);

	pdAngle[0] = atan2(s1, c1);
	pdAngle[1] = atan2(s2, c2);
}

void CRobotExp_4Dlg::OnBnClickedButtonForward()
{
	char cTmp[10];
	double dTmp[2];
	m_editTarPos1.GetWindowTextA(cTmp, 10);
	dTmp[0] = atof(cTmp);
	m_editTarPos2.GetWindowTextA(cTmp, 10);
	dTmp[1] = atof(cTmp);
	DataType_t jointData;
	GET_SYSTEM_MEMORY("JointData", jointData);
	jointData.Q_tar[0] = dTmp[0] * DEG2RAD;
	jointData.Q_tar[1] = dTmp[1] * DEG2RAD;
	SET_SYSTEM_MEMORY("JointData", jointData);
	double dPos[3] = { 0, 0, 0 };
	SolveForwardKinematics(dTmp[0]*DEG2RAD, dTmp[1]*DEG2RAD, dPos);
	char pszTmp[512];
	sprintf_s(pszTmp, "%.2lf", dPos[0]);
	m_editTarX.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.2lf", dPos[1]);
	m_editTarY.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.2lf", dPos[2]);
	m_editTarZ.SetWindowTextA(pszTmp);
}
void CRobotExp_4Dlg::OnBnClickedButtonInverse()
{
	char cTmp[10]; double dTmp[3];
	// Target Pose에 대한 정보를 Edit에서 가져옴
	m_editTarX.GetWindowTextA(cTmp, 10);
	dTmp[0] = atof(cTmp);
	m_editTarY.GetWindowTextA(cTmp, 10);
	dTmp[1] = atof(cTmp);
	m_editTarZ.GetWindowTextA(cTmp, 10);
	dTmp[2] = atof(cTmp);
	double dAngle[2] = { 0, 0 };
	// Inverse Kinematics를 품
	SolveInverseKinematics(dTmp[0], dTmp[1], dTmp[2], dAngle);
	char pszTmp[512];
	sprintf_s(pszTmp, "%.2lf", dAngle[0] * RAD2DEG);
	m_editTarPos1.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.2lf", dAngle[1] * RAD2DEG);
	m_editTarPos2.SetWindowTextA(pszTmp);
	DataType_t jointData;
	// Target 각도 Shared Memory에 저장
	GET_SYSTEM_MEMORY("JointData", jointData);
	jointData.Q_tar[0] = dAngle[0];
	jointData.Q_tar[1] = dAngle[1];
	SET_SYSTEM_MEMORY("JointData", jointData);
}

void CRobotExp_4Dlg::OnBnClickedButtonInit()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	DataType_t jointData;
	ControlData_t motor_data;
	GET_SYSTEM_MEMORY("JointData", jointData);
	jointData.Q_tar[0] = 0.0;
	jointData.Q_tar[1] = 0.0;
	SET_SYSTEM_MEMORY("JointData", jointData);

	motor_data.position = 0.0;
	motor_data.velocity = 10 * DEG2RAD;
	motor_data.current = 0.1 / 0.0683;
	SET_SYSTEM_MEMORY("Comm1Work_Controller_Target", motor_data);

	m_editTarPos1.SetWindowTextA("0.0");
	m_editTarPos2.SetWindowTextA("0.0");
	m_editTarX.SetWindowTextA("0.0");
	m_editTarY.SetWindowTextA("0.0");
	m_editTarZ.SetWindowTextA("0.0");

	m_editCurPos1.SetWindowTextA("0.0");
	m_editCurPos2.SetWindowTextA("0.0");
	m_editCurX.SetWindowTextA("0.0");
	m_editCurY.SetWindowTextA("0.0");
	m_editCurZ.SetWindowTextA("0.0");

	m_editCurVel.SetWindowTextA("0.0");
	m_editTarVel.SetWindowTextA("0.0");
	m_editCurTorq.SetWindowTextA("0.0");
	m_editTarTorq.SetWindowTextA("0.0");
	
}


void CRobotExp_4Dlg::OnTimer(UINT_PTR nIDEvent)
{
	DataType_t jointData;
	ControlData_t motor_data_cur;
	//ControlData_t motor_data_tar;
	CString str;

	GET_SYSTEM_MEMORY("JointData", jointData);
	GET_SYSTEM_MEMORY("Comm1Work_Controller_Current", motor_data_cur);
	//GET_SYSTEM_MEMORY("Comm1Work_Controller_Target", motor_data_tar);
	//GET_SYSTEM_MEMORY("Comm1Work_Controller_Target", motor_data);

	char pszTmp[512]; 
	// Shared Memory의 Current Q1, Q2값을 Edit창에 Update
	sprintf_s(pszTmp, "%.2lf", jointData.Q_cur[0] * RAD2DEG); 
	m_editCurPos1.SetWindowTextA(pszTmp); 
	sprintf_s(pszTmp, "%.2lf", jointData.Q_cur[1] * RAD2DEG);
	m_editCurPos2.SetWindowTextA(pszTmp);

	// Shared Memory의 Current Q1, 속도, 토크값을 Edit창에 Update
	str.Format("%.2lf", motor_data_cur.position * RAD2DEG / 1000);
	m_editCurPos1.SetWindowTextA(str);
	/*str.Format("%.2lf", motor_data.velocity * RAD2DEG /1000);*/
	str.Format("%.2lf", motor_data_cur.velocity * RAD2DEG / 1000);
	m_editCurVel.SetWindowTextA(str);
	/*str.Format("%.2lf", motor_data.current / 0.0683 /1000);*/
	str.Format("%.2lf", motor_data_cur.current * 0.0683 / 1000);
	m_editCurTorq.SetWindowTextA(str);

	// Forward Kinematics
	double dTmp[3] = { 0, 0, 0 }; 
	SolveForwardKinematics(jointData.Q_cur[0] /* RAD2DEG*/, jointData.Q_cur[1] /* RAD2DEG*/, dTmp); 
	//SolveForwardKinematics(motor_data_cur.position * RAD2DEG / 1000, jointData.Q_cur[1] /* RAD2DEG*/, dTmp);
	sprintf_s(pszTmp, "%.2lf", dTmp[0]);
	m_editCurX.SetWindowTextA(pszTmp); 
	sprintf_s(pszTmp, "%.2lf", dTmp[1]);
	m_editCurY.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.2lf", dTmp[2]);
	m_editCurZ.SetWindowTextA(pszTmp);


	CDialogEx::OnTimer(nIDEvent);
}


void CRobotExp_4Dlg::OnDestroy()
{
	CDialogEx::OnDestroy();

	delete m_pGraphDlg;
}

void CRobotExp_4Dlg::OnBnClickedButtonGraph()
{
	BOOL bCheck = m_pGraphDlg->IsWindowVisible(); 
	if (bCheck) m_pGraphDlg->ShowWindow(SW_HIDE); 
	else m_pGraphDlg->ShowWindow(SW_SHOW);
}


void CRobotExp_4Dlg::OnCbnDropdownComboPort()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CDeviceListReader reader;
	std::vector<std::string> list;

	m_ComboPort.ResetContent();

	reader.UpdateDeviceList("SERIALCOMM");
	reader.GetDeviceList(list);
	for (int i = 0; i < list.size(); i++) {
		m_ComboPort.AddString(list[i].c_str());
	}
}


void CRobotExp_4Dlg::OnBnClickedCheckOpen()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (m_CheckOpen.GetCheck()) {
		CString port, baud;
		m_ComboPort.GetLBText(m_ComboPort.GetCurSel(), port);
		m_ComboBaud.GetLBText(m_ComboBaud.GetCurSel(), baud);
		

		//if (m_comm.Open(port.GetBuffer(), atoi(baud.GetBuffer()))) {
		//	m_CheckOpen.SetWindowTextA("close");
		//}
		//else {
		//	AfxMessageBox("can't open port");
		//	m_CheckOpen.SetCheck(false);
		//}
		int nTmp = atoi(baud.GetBuffer());
		if (((CCommWork*)_commWorker.GetWork())->OpenPort(port.GetBuffer(),nTmp)){
			_commWorker.StartWork();
			m_CheckOpen.SetWindowText("Close");
		}
		else {
			AfxMessageBox("Can't open Port");
			m_CheckOpen.SetCheck(false);
		}

	}
	else {
		
		/*m_comm.Close();*/
		
		//_commWorker.StopWork();
		//((CCommWork*)_commWorker.GetWork())->ClosePort();



		m_CheckOpen.SetWindowText("Open");
	}
}



void CRobotExp_4Dlg::OnBnClickedButtonSet()
{
	ControlData_t motor_data;
	DataType_t ode_data;

	GET_SYSTEM_MEMORY("JointData", ode_data);

	CString str;

	m_editTarPos1.GetWindowTextA(str);
	ode_data.Q_tar[0] = atof(str.GetBuffer()) * DEG2RAD;

	m_editTarPos2.GetWindowTextA(str);
	ode_data.Q_tar[1] = atof(str.GetBuffer()) * DEG2RAD;

	//Target Pos1 값과 속도 토크를 전달 
	m_editTarPos1.GetWindowTextA(str);
	motor_data.position = atof(str.GetBuffer()) * DEG2RAD;

	m_editTarVel.GetWindowTextA(str);
	motor_data.velocity = atof(str.GetBuffer()) * DEG2RAD;

	m_editTarTorq.GetWindowTextA(str);
	motor_data.current = atof(str.GetBuffer()) / 0.0683;

	SET_SYSTEM_MEMORY("JointData", ode_data);
	SET_SYSTEM_MEMORY("Comm1Work_Controller_Target", motor_data);
}
