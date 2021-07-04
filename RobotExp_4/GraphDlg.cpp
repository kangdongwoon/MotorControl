// GraphDlg.cpp : 구현 파일입니다.
//

#include "stdafx.h"
#include "RobotExp_4.h"
#include "GraphDlg.h"
#include "afxdialogex.h"
#include "SystemMemory.h"
#include "DataType.h"

#define WHITE	0xffffff
#define BLACK	0x000000
#define RED		0x0000ff
#define BLUE	0xff0000

// CGraphDlg 대화 상자입니다.

IMPLEMENT_DYNAMIC(CGraphDlg, CDialogEx)

CGraphDlg::CGraphDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_GRAPH_DIALOG, pParent), m_dCnt(0)
{

}

CGraphDlg::~CGraphDlg()
{
}

void CGraphDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_NTGRAPH_POS, m_ntgPos);
	DDX_Control(pDX, IDC_NTGRAPH_VEL, m_ntgVel);
	DDX_Control(pDX, IDC_NTGRAPH_TORQ, m_ntgTor);
}


BEGIN_MESSAGE_MAP(CGraphDlg, CDialogEx)
	ON_WM_TIMER()
	
END_MESSAGE_MAP()


// CGraphDlg 메시지 처리기입니다.


void CGraphDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	m_dCnt += 0.1; 
	DataType_t jointData; 
	ControlData_t motor_data_cur;
	ControlData_t motor_data_tar;
	GET_SYSTEM_MEMORY("JointData", jointData);
	GET_SYSTEM_MEMORY("Comm1Work_Controller_Current", motor_data_cur);
	GET_SYSTEM_MEMORY("Comm1Work_Controller_Target", motor_data_tar);
	if (m_dCnt >= 10.0) { 
		m_ntgPos.SetRange(m_dCnt - 10.0, m_dCnt, -180.0, 180.0);
	}
	m_ntgPos.PlotXY(m_dCnt, jointData.Q_tar[0] * RAD2DEG, 1); 
	//m_ntgPos.PlotXY(m_dCnt, jointData.Q_cur[0] * RAD2DEG, 2);
	m_ntgPos.PlotXY(m_dCnt, motor_data_cur.position * RAD2DEG / 1000, 2);

	if (m_dCnt >= 10.0) {
		m_ntgVel.SetRange(m_dCnt - 10.0, m_dCnt, -180.0, 180.0);
	}
	//m_ntgVel.PlotXY(m_dCnt, motor_data_tar.velocity * RAD2DEG / 1000, 1);
	m_ntgVel.PlotXY(m_dCnt, motor_data_tar.velocity * RAD2DEG, 1);
	m_ntgVel.PlotXY(m_dCnt, -motor_data_tar.velocity * RAD2DEG, 2);
	m_ntgVel.PlotXY(m_dCnt, motor_data_cur.velocity * RAD2DEG / 1000, 3);

	if (m_dCnt >= 10.0) {
		m_ntgTor.SetRange(m_dCnt - 10.0, m_dCnt, -0.2, 0.2);
	}
	//m_ntgTor.PlotXY(m_dCnt, motor_data_tar.current * 0.0683 / 1000, 1);
	m_ntgTor.PlotXY(m_dCnt, motor_data_tar.current * 0.0683, 1);
	m_ntgTor.PlotXY(m_dCnt, -motor_data_tar.current * 0.0683, 2);
	m_ntgTor.PlotXY(m_dCnt, motor_data_cur.current * 0.0683 / 1000, 3);
	CDialogEx::OnTimer(nIDEvent);
}


void CGraphDlg::InitNTGraph()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.

	m_ntgPos.ClearGraph();
	m_ntgPos.ClearGraph(); 
	m_ntgVel.ClearGraph();
	m_ntgTor.ClearGraph();
	m_ntgPos.SetFrameStyle(0);
	m_ntgVel.SetFrameStyle(0);
	m_ntgTor.SetFrameStyle(0);
	m_ntgPos.SetPlotAreaColor(WHITE);
	m_ntgVel.SetPlotAreaColor(WHITE);
	m_ntgTor.SetPlotAreaColor(WHITE);
	m_ntgPos.SetShowGrid(TRUE);
	m_ntgVel.SetShowGrid(TRUE); 
	m_ntgTor.SetShowGrid(TRUE);
	m_ntgPos.SetFormatAxisBottom(_T("%.2f"));
	m_ntgVel.SetFormatAxisBottom(_T("%.2f"));
	m_ntgTor.SetFormatAxisBottom(_T("%.2f"));
	

	m_ntgPos.SetCaption(_T("위치"));
	m_ntgVel.SetCaption(_T("속도"));
	m_ntgTor.SetCaption(_T("토크"));
	m_ntgPos.SetXLabel(_T("Time[s]"));
	m_ntgVel.SetXLabel(_T("Time[s]"));
	m_ntgTor.SetXLabel(_T("Time[s]"));
	m_ntgPos.SetYLabel(_T("Degree[deg]"));
	m_ntgVel.SetYLabel(_T("Velocity[deg/s]"));
	m_ntgTor.SetYLabel(_T("Torque[Nm]"));
	m_ntgPos.AddElement();
	m_ntgPos.SetElementWidth(3);
	m_ntgPos.SetElementLineColor(RED);// Target
	m_ntgPos.AddElement();
	m_ntgPos.SetElementWidth(3);
	m_ntgPos.SetElementLineColor(BLUE); // Current

	m_ntgPos.SetRange(0.0, 10.0, -180.0, 180.0);
	m_ntgPos.SetYGridNumber(4);
	m_ntgVel.SetYGridNumber(4);
	m_ntgTor.SetYGridNumber(4);

	m_ntgVel.AddElement();
	m_ntgVel.SetElementWidth(4);
	m_ntgVel.SetElementLineColor(RED);// Target
	m_ntgVel.AddElement();
	m_ntgVel.SetElementWidth(4);
	m_ntgVel.SetElementLineColor(RED);// Target
	m_ntgVel.AddElement();
	m_ntgVel.SetElementWidth(3);
	m_ntgVel.SetElementLineColor(BLUE); // Current

	m_ntgTor.AddElement();
	m_ntgTor.SetElementWidth(4);
	m_ntgTor.SetElementLineColor(RED);// Target
	m_ntgTor.AddElement();
	m_ntgTor.SetElementWidth(4);
	m_ntgTor.SetElementLineColor(RED);// Target
	m_ntgTor.AddElement();
	m_ntgTor.SetElementWidth(3);
	m_ntgTor.SetElementLineColor(BLUE); // Current

	SetTimer(1, 100, NULL);
}




BOOL CGraphDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  여기에 추가 초기화 작업을 추가합니다.
	InitNTGraph();

	return TRUE;  // return TRUE unless you set the focus to a control
				  // 예외: OCX 속성 페이지는 FALSE를 반환해야 합니다.
}
