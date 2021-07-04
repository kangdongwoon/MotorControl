#pragma once
#include "NTGraph.h"
//#include "COCX.h"


// CGraphDlg 대화 상자입니다.

class CGraphDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CGraphDlg)

public:
	CGraphDlg(CWnd* pParent = NULL);   // 표준 생성자입니다.
	virtual ~CGraphDlg();

// 대화 상자 데이터입니다.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_GRAPH_DIALOG };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

	DECLARE_MESSAGE_MAP()
public:

	CNTGraph m_ntgPos;
	CNTGraph m_ntgVel;
	CNTGraph m_ntgTor;
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void InitNTGraph();
	CNTGraph m_ntgTest;
	virtual BOOL OnInitDialog();
	double m_dCnt;
};
