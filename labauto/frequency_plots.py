from __future__ import annotations

import numpy as np
from scipy import signal
import plotly.graph_objects as go
from plotly.subplots import make_subplots


def _next_pow2(n: int) -> int:
    """Return next power of 2 >= n (with n >= 1)."""
    n = int(n)
    if n < 1:
        n = 1
    return 1 << int(np.ceil(np.log2(n)))

def plot_frf(
    y: np.ndarray,
    u: np.ndarray,
    fs: float,
    *,
    existing_fig_frf: go.Figure | None = None,
    trace_name: str | None = None,
    nperseg: int = 2048,
    noverlap: int | None = None,
    nfft: int | None = None,
    detrend: str | None = "constant",
    window: str = "hann",
    eps: float = 1e-12,
    max_freq: float | None = None,
    show_coherence: bool = False,
):
    """
    Estimate and plot the H1 FRF using Welch/CPSD:

        H1(f) = S_yu(f) / S_uu(f)

    If existing_fig_frf is provided, add traces to it (overlay).
    Assumes existing figure uses rows:
      row 1: magnitude, row 2: phase, row 3: coherence (optional).

    Returns
    -------
    fig_frf : plotly.graph_objects.Figure
    frf : dict
        Keys: 'f', 'H', 'mag_db', 'phase_deg', 'coh_f', 'coh'
    """
    y = np.asarray(y).squeeze()
    u = np.asarray(u).squeeze()

    if y.ndim != 1 or u.ndim != 1:
        raise ValueError("y and u must be 1D arrays after squeeze().")
    if y.size != u.size:
        raise ValueError(f"y and u must have the same length. Got {y.size=} and {u.size=}.")
    if fs <= 0:
        raise ValueError("fs must be > 0.")
    if nperseg < 2:
        raise ValueError("nperseg must be >= 2.")
    if noverlap is None:
        noverlap = nperseg // 2
    if not (0 <= noverlap < nperseg):
        raise ValueError("noverlap must satisfy 0 <= noverlap < nperseg.")
    if nfft is None:
        nfft = _next_pow2(nperseg)
    if nfft < nperseg:
        raise ValueError("nfft must be >= nperseg.")

    # --- Auto-spectrum of input: S_uu
    f, Puu = signal.welch(
        u,
        fs=fs,
        window=window,
        nperseg=nperseg,
        noverlap=noverlap,
        nfft=nfft,
        detrend=detrend,
        scaling="density",
        return_onesided=True,
    )

    # --- Cross-spectrum: S_yu (note: csd(x, y) estimates S_xy)
    f2, Pyu = signal.csd(
        y,
        u,
        fs=fs,
        window=window,
        nperseg=nperseg,
        noverlap=noverlap,
        nfft=nfft,
        detrend=detrend,
        scaling="density",
        return_onesided=True,
    )

    if not np.allclose(f, f2):
        raise RuntimeError("Frequency grids mismatch between welch and csd (unexpected).")

    # --- H1 FRF estimate
    H = Pyu / (Puu + eps)
    mag_db = 20.0 * np.log10(np.abs(H) + eps)
    phase_deg = np.unwrap(np.angle(H)) * 180.0 / np.pi

    # --- Coherence γ²_yu
    f_coh, coh = signal.coherence(
        y,
        u,
        fs=fs,
        window=window,
        nperseg=nperseg,
        noverlap=noverlap,
        nfft=nfft,
        detrend=detrend,
    )

    # --- Optional frequency limit
    if max_freq is not None:
        m = f <= max_freq
        f, H, mag_db, phase_deg = f[m], H[m], mag_db[m], phase_deg[m]

        mc = f_coh <= max_freq
        f_coh, coh = f_coh[mc], coh[mc]

    # For log x-axis, remove DC (0 Hz) everywhere consistently
    mpos = f > 0
    f_plot = f[mpos]
    mag_plot = mag_db[mpos]
    phase_plot = phase_deg[mpos]

    if show_coherence:
        mposc = f_coh > 0
        f_coh_plot = f_coh[mposc]
        coh_plot = coh[mposc]

    # --- Figure creation / reuse
    if trace_name is None:
        trace_name = f"FRF (nperseg={nperseg}, nfft={nfft})"

    if existing_fig_frf is None:
        rows = 3 if show_coherence else 2
        titles = (
            ["Magnitude |H(f)| [dB]", "Phase ∠H(f) [deg]"]
            + (["Coherence γ²_yu(f)"] if show_coherence else [])
        )

        fig = make_subplots(
            rows=rows,
            cols=1,
            shared_xaxes=True,
            vertical_spacing=0.08,
            subplot_titles=titles,
        )

        # Axes formatting
        for r in range(1, rows + 1):
            fig.update_xaxes(type="log", row=r, col=1)

        fig.update_yaxes(title_text="dB", row=1, col=1)
        fig.update_yaxes(title_text="deg", row=2, col=1)
        fig.update_xaxes(title_text="Frequency [Hz]", row=rows, col=1)

        if show_coherence:
            fig.update_yaxes(title_text="γ²", range=[0, 1], row=3, col=1)

        fig.update_layout(
            title="Estimated FRF (H1 = S_yu / S_uu)" + (" + coherence" if show_coherence else ""),
            legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="left", x=0),
            height=900 if show_coherence else 700,
        )
    else:
        fig = existing_fig_frf

        # If user wants coherence but the existing figure likely has only 2 rows,
        # we can’t safely “add a row” in-place. Fail fast with a clear message.
        if show_coherence:
            # Best-effort heuristic: if any trace is assigned to row 3, it exists.
            has_row3 = any(getattr(tr, "yaxis", "") in ("y3", "yaxis3") for tr in fig.data)
            # Another heuristic: layout has yaxis3
            has_yaxis3 = hasattr(fig.layout, "yaxis3") and fig.layout.yaxis3 is not None
            if not (has_row3 or has_yaxis3):
                raise ValueError(
                    "show_coherence=True but existing_fig_frf does not appear to have a 3rd subplot row. "
                    "Create the figure initially with show_coherence=True, then reuse it."
                )

    # --- Add traces (always overlay)
    fig.add_trace(
        go.Scatter(x=f_plot, y=mag_plot, mode="lines", name=f"{trace_name} |H| [dB]"),
        row=1,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=f_plot, y=phase_plot, mode="lines", name=f"{trace_name} phase [deg]"),
        row=2,
        col=1,
    )

    if show_coherence:
        fig.add_trace(
            go.Scatter(x=f_coh_plot, y=coh_plot, mode="lines", name=f"{trace_name} coherence"),
            row=3,
            col=1,
        )

    frf = {
        "f": f,
        "H": H,
        "mag_db": mag_db,
        "phase_deg": phase_deg,
        "coh_f": f_coh,
        "coh": coh,
    }
    return fig, frf


def plot_spectrogram(
    y: np.ndarray,
    fs: float,
    *,
    nperseg: int = 2048,
    noverlap: int | None = None,
    nfft: int | None = None,
    detrend: str | None = "constant",
    window: str = "hann",
    eps: float = 1e-12,
    max_freq: float | None = None,
):
    """
    Plot a spectrogram of y using STFT magnitude (dB) as a Plotly heatmap.

    Parameters
    ----------
    y : np.ndarray
        Signal to analyze (1D after squeeze()).
    fs : float
        Sampling frequency [Hz], must be > 0.
    nperseg : int
        Segment length for STFT.
    noverlap : int | None
        Overlap in samples. Default: nperseg//2.
    nfft : int | None
        FFT length. Default: next power of 2 >= nperseg.
    detrend : str | None
        Detrend setting passed to scipy.signal.stft.
    window : str
        Window name (e.g., 'hann').
    eps : float
        Small positive constant to avoid log(0).
    max_freq : float | None
        If set, limit plot to [0, max_freq] Hz.

    Returns
    -------
    fig_spec : plotly.graph_objects.Figure
    """
    y = np.asarray(y).squeeze()

    if y.ndim != 1:
        raise ValueError("y must be a 1D array after squeeze().")
    if fs <= 0:
        raise ValueError("fs must be > 0.")
    if nperseg < 2:
        raise ValueError("nperseg must be >= 2.")
    if noverlap is None:
        noverlap = nperseg // 2
    if not (0 <= noverlap < nperseg):
        raise ValueError("noverlap must satisfy 0 <= noverlap < nperseg.")
    if nfft is None:
        nfft = _next_pow2(nperseg)
    if nfft < nperseg:
        raise ValueError("nfft must be >= nperseg.")

    f, t, Zxx = signal.stft(
        y,
        fs=fs,
        window=window,
        nperseg=nperseg,
        noverlap=noverlap,
        nfft=nfft,
        detrend=detrend,
        boundary=None,
        padded=False,
        return_onesided=True,
    )

    S_db = 20.0 * np.log10(np.abs(Zxx) + eps)

    if max_freq is not None:
        m = f <= max_freq
        f = f[m]
        S_db = S_db[m, :]

    fig = go.Figure(
        data=go.Heatmap(
            x=t,
            y=f,
            z=S_db,
            colorbar=dict(title="dB"),
        )
    )
    fig.update_layout(
        title="Spectrogram of y (STFT magnitude in dB)",
        xaxis_title="Time [s]",
        yaxis_title="Frequency [Hz]",
    )
    return fig

