
// ImageProcessingDlg.h : header file
//

#pragma once


// CImageProcessingDlg dialog
class CImageProcessingDlg : public CDialogEx
{
// Construction
public:
	CImageProcessingDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_IMAGEPROCESSING_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support

// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
	public:
	afx_msg void OnBnClickedLinedetection();

	afx_msg void OnBnClickeddecomposition();
	afx_msg void OnBnClickedequalizehist();
	afx_msg void OnBnClickedaveragedepth();
};
